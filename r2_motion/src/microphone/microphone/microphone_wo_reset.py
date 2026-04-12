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

# -----------------------
class Microphone(Node):
    def __init__(self):
        super().__init__('microhone_node')

        self.pub_doa = self.create_publisher(Float32, '/microphone/doa', 10)
        self.pub_stt = self.create_publisher(String, '/microphone/stt', 10)

        # Wake word
        self.oww_model = Model()

        # STT
        self.whisper = WhisperModel("tiny", compute_type="int8")

        # Audio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
        )

        # DOA
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x001A)
        if self.dev:
            self.get_logger().debug("Resetting Respeaker USB device.")
            self.dev.reset()
            time.sleep(3)
        self.RESID = 20
        self.CMDID = 0x80 | 18
        self.LENGTH = 5

        self.trigger_count = 0

        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info("Microphone node started")

    # -----------------------
    def get_doa(self):
        data = self.dev.ctrl_transfer(0xC0, 0, self.CMDID, self.RESID, self.LENGTH, timeout=1000)
        raw = data[1] | (data[2] << 8)
        return (raw - 270 + 180) % 360 - 180

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
            data = self.stream.read(CHUNK, exception_on_overflow=False)
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
        data = self.stream.read(CHUNK, exception_on_overflow=False)
        pcm = np.frombuffer(data, dtype=np.int16)

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

            msg1 = Float32()
            msg1.data = float(doa)
            msg2 = String()
            msg2.data = text

            self.pub_doa.publish(msg1)
            self.pub_stt.publish(msg2)

            self.get_logger().info(f"DOA: {doa:.1f} | TEXT: {text}")

            time.sleep(1.5)

# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = Microphone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()