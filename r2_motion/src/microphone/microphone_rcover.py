import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

import openwakeword
from openwakeword.model import Model
import pyaudio
import numpy as np
from faster_whisper import WhisperModel
import wave
import time
import usb.core

# -----------------------
# CONFIG
# -----------------------
CHUNK = 1280
RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16

TARGET = "hey_jarvis"
THRESHOLD = 0.5
REQUIRED_FRAMES = 3

VAD_THRESHOLD = 500
SILENCE_DURATION = 1.2
MAX_WAIT_FOR_SPEECH = 3.0
MIN_SPEECH_DURATION = 0.5

DEAD_AUDIO_THRESHOLD = 5

# -----------------------
def find_respeaker_device(p):
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        if "ReSpeaker" in dev['name'] or "USB Audio" in dev['name']:
            return i
    return None

# -----------------------
class WakeNode(Node):
    def __init__(self):
        super().__init__('wake_node')
  
        # ✅ Publishers
        self.doa_pub = self.create_publisher(Float32, '/microphone/doa', 10)
        self.stt_pub = self.create_publisher(String, '/microphone/stt', 10)

        self.oww_model = Model()
        self.whisper = WhisperModel("tiny", compute_type="int8")

        self.audio = None
        self.stream = None

        # DOA setup
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x001A)
        self.RESID = 20
        self.CMDID = 0x80 | 18
        self.LENGTH = 5

        self.trigger_count = 0
        
        self.start_audio_with_retry()
      
        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info("Wake node started")

    # -----------------------
    def start_audio_with_retry(self):
        for i in range(5):
            self.get_logger().info(f"Attempting audio init ({i+1}/5)...")
            if self.restart_audio():
                return
            time.sleep(2)

        raise RuntimeError("Failed to initialize audio")

    # -----------------------
    def restart_audio(self):
        self.get_logger().warn("Restarting audio stream...")

        if hasattr(self, "stream") and self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except:
                pass

        if hasattr(self, "audio") and self.audio:
            try:
                self.audio.terminate()
            except:
                pass

        time.sleep(1)

        try:
            self.audio = pyaudio.PyAudio()
            device_index = find_respeaker_device(self.audio)

            if device_index is None:
                self.get_logger().warn("ReSpeaker not found")
                return False

            self.stream = self.audio.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK
            )

            self.get_logger().info(f"Audio initialized on device {device_index}")
            return True

        except Exception as e:
            self.get_logger().warn(f"Audio init failed: {e}")
            return False

    # -----------------------
    def get_doa(self):
        try:
            data = self.dev.ctrl_transfer(0xC0, 0, self.CMDID, self.RESID, self.LENGTH, timeout=1000)
            raw = data[1] | (data[2] << 8)
            return (raw - 270 + 180) % 360 - 180
        except:
            return float('nan')

    # -----------------------
    def record_audio_vad(self):
        frames = []
        silence_chunks = 0
        speech_chunks = 0
        waited_chunks = 0

        max_silence_chunks = int(SILENCE_DURATION * RATE / CHUNK)
        max_wait_chunks = int(MAX_WAIT_FOR_SPEECH * RATE / CHUNK)
        min_speech_chunks = int(MIN_SPEECH_DURATION * RATE / CHUNK)
        max_record_chunks = int(6.0 * RATE / CHUNK)

        recording = False
        total_chunks = 0

        while True:
            try:
                data = self.stream.read(CHUNK, exception_on_overflow=False)
            except:
                return None

            pcm = np.frombuffer(data, dtype=np.int16)
            energy = np.abs(pcm).mean()

            total_chunks += 1

            if total_chunks > max_record_chunks:
                break

            if not recording:
                waited_chunks += 1

                if energy > VAD_THRESHOLD:
                    recording = True
                    frames.append(data)
                    speech_chunks += 1
                    continue

                if waited_chunks > max_wait_chunks:
                    return None

            else:
                frames.append(data)

                if energy > VAD_THRESHOLD:
                    silence_chunks = 0
                    speech_chunks += 1
                else:
                    silence_chunks += 1

                    if silence_chunks > max_silence_chunks and speech_chunks > min_speech_chunks:
                        break

        if len(frames) == 0:
            return None

        filename = "command.wav"
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))

        return filename

    # -----------------------
    def loop(self):
        try:
            data = self.stream.read(CHUNK, exception_on_overflow=False)
        except Exception as e:
            self.get_logger().warn(f"Stream read failed: {e}")
            self.restart_audio()
            return

        pcm = np.frombuffer(data, dtype=np.int16)
        energy = np.abs(pcm).mean()

        if energy < DEAD_AUDIO_THRESHOLD:
            self.get_logger().warn("Audio appears dead — restarting")
            self.restart_audio()
            return

        prediction = self.oww_model.predict(pcm)
        score = prediction.get(TARGET, 0)

        if score > THRESHOLD:
            self.trigger_count += 1
        else:
            self.trigger_count = 0

        if self.trigger_count >= REQUIRED_FRAMES:
            self.get_logger().info("Wake word detected")
            self.trigger_count = 0

            doa = self.get_doa()

            filename = self.record_audio_vad()
            if filename is None:
                return

            segments, _ = self.whisper.transcribe(filename)
            text = " ".join([s.text for s in segments])

            # ✅ Publish DOA
            doa_msg = Float32()
            doa_msg.data = float(doa)
            self.doa_pub.publish(doa_msg)

            # ✅ Publish STT
            stt_msg = String()
            stt_msg.data = text
            self.stt_pub.publish(stt_msg)

            self.get_logger().info(f"DOA: {doa:.1f} | TEXT: {text}")

            time.sleep(1.5)

# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = WakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()