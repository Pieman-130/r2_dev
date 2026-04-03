# respeaker_msgs/msg/AudioStamped.msg
#
# VAD-gated raw PCM audio from the ReSpeaker XVF3800.
# One message = one PyAudio chunk (default: 1024 frames ≈ 64 ms at 16 kHz).

std_msgs/Header header    # ROS timestamp + frame_id ("respeaker")

string   encoding         # Sample format: "S16LE" (signed 16-bit, little-endian)
uint8    channels         # Number of channels: 1 (mono, processed/ASR output)
uint32   sample_rate      # Sample rate in Hz: 16000
uint8[]  data             # Raw PCM bytes (int16 samples packed little-endian)