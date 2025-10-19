import subprocess

def speak_festival_with_voice(text, voice="kal_diphone"):
    """Speak text using a specific Festival voice."""
    command = f'(voice_{voice}) (SayText "{text}")'
    subprocess.run(['festival', '--pipe'], input=command, text=True)

# Example usage
speak_festival_with_voice("Hello from Festival using mbrola voice!", "us2_mbrola")
speak_festival_with_voice("Hello from Festival using SLT voice!", "cmu_us_slt_arctic_hts")

