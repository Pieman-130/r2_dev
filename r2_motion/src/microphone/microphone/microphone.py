"""
respeaker_node.py
=================
ROS2 Jazzy node for the Seeed ReSpeaker XVF3800 USB 4-Mic Array.

Published topics
----------------
/respeaker/doa         geometry_msgs/Twist
    Direction of Arrival, published continuously at `doa_rate_hz`.
    linear.x  = cos(azimuth_rad)   X component of unit vector toward speaker
    linear.y  = sin(azimuth_rad)   Y component of unit vector toward speaker
    angular.z = azimuth_rad        azimuth (0 = USB-connector end, CCW+)

/respeaker/audio       respeaker_xvf3800_msgs/AudioStamped
    VAD-gated raw PCM audio. Only published while speech is detected.
    encoding="S16LE", channels=1, sample_rate=16000.

Hardware interface
------------------
DOA / VAD  : pyusb  — USB vendor control transfers to the XVF3800
Audio      : pyaudio — USB UAC1 audio device (same physical hardware)

System dependencies
-------------------
    sudo apt install python3-pyaudio portaudio19-dev
    pip install pyusb

ROS2 dependencies (all in-workspace or standard Jazzy packages)
---------------------------------------------------------------
    rclpy  geometry_msgs  std_msgs  respeaker_xvf3800_msgs (this workspace)
"""

import math
import struct
import threading
import time

import pyaudio
import usb.core
import usb.util

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from geometry_msgs.msg import Twist
from respeaker_msgs.msg import AudioStamped


# ── XVF3800 USB identifiers ────────────────────────────────────────────────
_VENDOR_ID  = 0x2886
_PRODUCT_ID = 0x001A   # Seeed reSpeaker XVF3800 USB

# ── XVF3800 USB parameter table ────────────────────────────────────────────
# (resid, cmdid, wire_byte_len, access, dtype)
#
#  resid        USB wIndex (resource / module id)
#  cmdid        USB wValue base; OR with 0x80 for reads
#  wire_byte_len total bytes returned on the wire, INCLUDING the leading
#               1-byte status/padding byte the firmware always prepends
#  dtype        "uint8" | "uint16" | "uint32" | "radians" (float32 LE)
_PARAMETERS: dict[str, tuple] = {
    "VERSION":             (48,  0,  4, "ro", "uint8"),
    # 4 float32 azimuths + 1 status byte = 17 bytes
    "AEC_AZIMUTH_VALUES":  (33, 75, 17, "ro", "radians"),
    # 4 uint32 speech-energy values + 1 status byte = 17 bytes
    "AEC_SPENERGY_VALUES": (33, 76, 17, "ro", "uint32"),
}

# ── Audio constants ────────────────────────────────────────────────────────
_SAMPLE_RATE  = 16_000   # Hz — XVF3800 native rate
_CHANNELS     = 1        # mono: beamformed / AEC processed output
_CHUNK_FRAMES = 1_024    # PyAudio frames per read (≈ 64 ms at 16 kHz)
_PA_FORMAT    = pyaudio.paInt16

# Substring matched against PyAudio device names to find the XVF3800
_DEVICE_NAME_HINT = "ReSpeaker"

# Default speech-energy threshold for the software VAD gate
_DEFAULT_VAD_ENERGY = 1_000_000.0


# ── XVF3800 USB low-level driver ───────────────────────────────────────────

class XVF3800USB:
    """
    Reads named parameters from the XVF3800 via pyusb vendor control
    transfers.  The firmware always prepends a 1-byte status/padding value
    before the actual payload; this class strips it transparently.
    """

    TIMEOUT_US = 100_000  # libusb timeout (µs)

    def __init__(self, dev: usb.core.Device) -> None:
        self._dev = dev

    def read(self, name: str) -> list:
        """Return a list of parsed values for the named parameter."""
        try:
            resid, cmdid, wire_bytes, _access, dtype = _PARAMETERS[name]
        except KeyError:
            raise KeyError(f"XVF3800: unknown parameter '{name}'")

        raw = bytes(
            self._dev.ctrl_transfer(
                usb.util.CTRL_IN
                | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                0,               # bRequest (always 0 for XVF3800)
                0x80 | cmdid,    # wValue: read-flag OR'd with command id
                resid,           # wIndex: resource / module id
                wire_bytes,      # wLength
                self.TIMEOUT_US,
            )
        )

        # Skip byte 0 — firmware status/padding byte
        payload = raw[1:]

        if dtype in ("radians", "float"):
            n = len(payload) // 4
            return list(struct.unpack_from(f"<{n}f", payload))
        elif dtype == "uint8":
            return list(raw)        # status byte is harmless here
        elif dtype == "uint16":
            n = len(payload) // 2
            return list(struct.unpack_from(f"<{n}H", payload))
        elif dtype == "uint32":
            n = len(payload) // 4
            return list(struct.unpack_from(f"<{n}I", payload))
        return list(raw)


# ── ROS2 node ──────────────────────────────────────────────────────────────

class ReSpeakerNode(Node):
    """
    ROS2 Jazzy node that reads DOA and audio from the ReSpeaker XVF3800 and
    publishes them as standard ROS2 messages.

    Architecture:
      • A ROS2 timer fires at `doa_rate_hz` Hz to poll DOA + speech energy
        over USB and publish geometry_msgs/Twist.
      • A daemon thread runs a tight PyAudio read loop and publishes
        AudioStamped when the VAD gate is open.
      • A single threading.Lock protects the shared `_speech_active` flag
        between the two threads.
    """

    def __init__(self) -> None:
        super().__init__("respeaker_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("doa_rate_hz",          10.0)
        self.declare_parameter("vad_energy_threshold", _DEFAULT_VAD_ENERGY)
        self.declare_parameter("audio_device_name",    _DEVICE_NAME_HINT)
        self.declare_parameter("sample_rate",          _SAMPLE_RATE)
        self.declare_parameter("chunk_frames",         _CHUNK_FRAMES)
        self.declare_parameter("frame_id",             "respeaker")
        # Mounting correction: degrees to add to raw azimuth.
        # 0   = USB connector points forward on robot
        # 90  = USB connector points left
        # 180 = USB connector points backward
        self.declare_parameter("doa_offset_deg",       0.0)

        doa_rate        = self.get_parameter("doa_rate_hz").value
        self._vad_thr   = self.get_parameter("vad_energy_threshold").value
        dev_hint        = self.get_parameter("audio_device_name").value
        self._rate      = self.get_parameter("sample_rate").value
        self._chunk     = self.get_parameter("chunk_frames").value
        self._frame_id  = self.get_parameter("frame_id").value
        offset_deg      = self.get_parameter("doa_offset_deg").value
        self._doa_offset = math.radians(offset_deg)

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._doa_pub   = self.create_publisher(Twist,         "respeaker/doa",   sensor_qos)
        self._audio_pub = self.create_publisher(AudioStamped,  "respeaker/audio", sensor_qos)

        # Latched topic: late-joining subscribers always receive stream info.
        # Publish a string description rather than a separate message type
        # to keep the dependency list minimal.
        from std_msgs.msg import String
        self._info_pub = self.create_publisher(String, "respeaker/audio_info", latched_qos)

        # ── USB device ────────────────────────────────────────────────────
        dev = usb.core.find(idVendor=_VENDOR_ID, idProduct=_PRODUCT_ID)
        if dev is None:
            self.get_logger().fatal(
                f"ReSpeaker XVF3800 not found "
                f"(VID=0x{_VENDOR_ID:04X} PID=0x{_PRODUCT_ID:04X}). "
                "Check USB connection and udev rules (see README)."
            )
            raise RuntimeError("XVF3800 USB device not found")

        self._xvf = XVF3800USB(dev)
        self.get_logger().info("XVF3800 USB control interface opened.")

        # ── Shared state ──────────────────────────────────────────────────
        self._speech_active = False
        self._lock = threading.Lock()

        # ── DOA timer ─────────────────────────────────────────────────────
        self._doa_timer = self.create_timer(1.0 / doa_rate, self._doa_cb)

        # ── Audio ─────────────────────────────────────────────────────────
        self._pa = pyaudio.PyAudio()
        dev_idx  = self._find_audio_device(dev_hint)
        self._stream = self._open_stream(dev_idx)

        self._audio_thread = threading.Thread(
            target=self._audio_loop, daemon=True, name="respeaker_audio"
        )
        self._audio_thread.start()

        # Publish static info (latched)
        from std_msgs.msg import String as Str
        info = Str()
        info.data = (
            f"encoding=S16LE channels={_CHANNELS} "
            f"sample_rate={self._rate} frame_id={self._frame_id}"
        )
        self._info_pub.publish(info)

        self.get_logger().info(
            f"ReSpeaker XVF3800 ready — DOA @ {doa_rate} Hz | "
            f"audio {self._rate} Hz S16LE mono | "
            f"VAD threshold {self._vad_thr:.0f} | "
            f"DOA offset {offset_deg:.1f}°"
        )

    # ── DOA timer callback (ROS executor thread) ───────────────────────────

    def _doa_cb(self) -> None:
        try:
            # AEC_AZIMUTH_VALUES → [beam0, beam1, beam2, auto-selected]
            # Index -1 is the auto-selected (best) beam.
            azimuths = self._xvf.read("AEC_AZIMUTH_VALUES")
            doa_rad  = float(azimuths[-1]) + self._doa_offset

            # Max energy across beams drives the software VAD gate.
            energies = self._xvf.read("AEC_SPENERGY_VALUES")
            active   = bool(max(energies, default=0) > self._vad_thr)

        except Exception as exc:
            self.get_logger().warn(
                f"XVF3800 USB read error: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        with self._lock:
            self._speech_active = active

        # Publish DOA as Twist
        # Encoding DOA three ways so downstream nodes have maximum flexibility:
        #   linear.x / linear.y → unit vector (useful for 2-D bearing control)
        #   angular.z           → raw angle in radians (useful for math)
        msg = Twist()
        msg.linear.x  = math.cos(doa_rad)
        msg.linear.y  = math.sin(doa_rad)
        msg.angular.z = doa_rad
        self._doa_pub.publish(msg)

    # ── Audio loop (daemon thread) ─────────────────────────────────────────

    def _audio_loop(self) -> None:
        while rclpy.ok():
            try:
                raw = self._stream.read(self._chunk, exception_on_overflow=False)
            except OSError as exc:
                self.get_logger().warn(
                    f"PyAudio read error: {exc}", throttle_duration_sec=5.0
                )
                time.sleep(0.02)
                continue

            with self._lock:
                should_publish = self._speech_active

            if should_publish:
                msg = AudioStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                msg.encoding        = "S16LE"
                msg.channels        = _CHANNELS
                msg.sample_rate     = self._rate
                msg.data            = list(raw)  # bytes → list[uint8]
                self._audio_pub.publish(msg)

    # ── Audio device helpers ───────────────────────────────────────────────

    def _find_audio_device(self, hint: str) -> int | None:
        count = self._pa.get_host_api_info_by_index(0)["deviceCount"]
        for i in range(count):
            info = self._pa.get_device_info_by_host_api_device_index(0, i)
            if hint.lower() in info["name"].lower() and info["maxInputChannels"] > 0:
                self.get_logger().info(f"Audio input device [{i}]: {info['name']}")
                return i
        self.get_logger().warn(
            f"No audio device matching '{hint}' found — using system default."
        )
        return None

    def _open_stream(self, device_index: int | None) -> pyaudio.Stream:
        kwargs: dict = dict(
            format=_PA_FORMAT,
            channels=_CHANNELS,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
        )
        if device_index is not None:
            kwargs["input_device_index"] = device_index
        return self._pa.open(**kwargs)

    # ── Cleanup ────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down ReSpeaker XVF3800 node…")
        try:
            self._stream.stop_stream()
            self._stream.close()
        except Exception:
            pass
        try:
            self._pa.terminate()
        except Exception:
            pass
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ReSpeakerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()