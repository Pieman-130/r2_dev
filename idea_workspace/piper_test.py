import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import urllib.request
import urllib.error
import json
import time


class PiperTTSNode(Node):
    def __init__(self):
        super().__init__('piper_tts')

        self.declare_parameter('model', '/home/adam/.local/share/piper/en_US-arctic-medium.onnx')
        self.declare_parameter('speaker_id', 11)
        self.declare_parameter('output_rate', 48000)
        self.declare_parameter('channels', 2)
        self.declare_parameter('device', 'hw:0,0')
        self.declare_parameter('piper_port', 5000)

        self._lock       = threading.Lock()
        self._piper_proc = None

        self._start_piper_server()

        self.create_subscription(String, '/tts/say', self._say_cb, 10)
        self.get_logger().info('Piper TTS node ready.')

    def _start_piper_server(self):
        model = self.get_parameter('model').value
        port  = self.get_parameter('piper_port').value

        self.get_logger().info(f'Starting Piper HTTP server on port {port}...')
        self._piper_proc = subprocess.Popen(
            ['python3', '-m', 'piper.http_server',
             '--model', model,
             '--port', str(port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Wait for server to be ready
        url = f'http://localhost:{port}/info'
        for _ in range(20):
            try:
                urllib.request.urlopen(url, timeout=1)
                self.get_logger().info('Piper HTTP server ready.')
                return
            except Exception:
                time.sleep(0.5)

        self.get_logger().error('Piper HTTP server failed to start!')

    def _say(self, text: str):
        port        = self.get_parameter('piper_port').value
        output_rate = self.get_parameter('output_rate').value
        channels    = self.get_parameter('channels').value
        device      = self.get_parameter('device').value
        speaker_id     = self.get_parameter('speaker_id').value

        # POST to /synthesize with JSON body
        payload_dict = {'text': text.strip()}
        if speaker_id >= 0:
            payload_dict['speaker_id'] = speaker_id

        payload = json.dumps(payload_dict).encode()
        req = urllib.request.Request(
            f'http://localhost:{port}/synthesize',
            data=payload,
            headers={'Content-Type': 'application/json'}
        )
        with urllib.request.urlopen(req) as response:
            wav_data = response.read()

        # WAV → sox resample → aplay
        sox = subprocess.Popen(
            ['sox', '-t', 'wav', '-',
             '-t', 'raw', '-r', str(output_rate), '-c', str(channels), '-'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        aplay = subprocess.Popen(
            ['aplay', '-r', str(output_rate), '-f', 'S16_LE', '-t', 'raw',
             '-c', str(channels), '-D', device, '-'],
            stdin=sox.stdout
        )

        sox.stdin.write(wav_data)
        sox.stdin.close()
        sox.wait()
        aplay.wait()

    def _say_cb(self, msg: String):
        with self._lock:
            try:
                self._say(msg.data)
            except Exception as e:
                self.get_logger().error(f'TTS error: {e}')

    def destroy_node(self):
        if self._piper_proc is not None:
            self._piper_proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = PiperTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()