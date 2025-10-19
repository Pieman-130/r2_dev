import subprocess

def speak_flite(text, voice="slt", volume=1.0, device="hw:1,0"):
    """
    Speak text using Flite TTS, resample to 48kHz stereo, and play on USB speaker.
    """
    wav_path = "/tmp/flite_output.wav"

    # 1️⃣ Generate WAV (Flite outputs 16kHz mono)
    subprocess.run(["flite", "-voice", voice, "-t", text, "-o", wav_path], check=True)

    # 2️⃣ Resample and upmix before sending to USB speaker
    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-i", wav_path,
        "-ar", "48000",         # resample to 48kHz
        "-ac", "2",             # upmix mono → stereo
        "-filter:a", f"volume={volume}",
        "-f", "alsa", device
    ], check=True)

# Example
speak_flite("Testing with stereo audio on USB!", voice="rms", volume=1.5, device="hw:1,0")
