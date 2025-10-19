import subprocess
from subprocess import call

def speak_flite(text):
    """Speaks the given text using flite."""
    subprocess.run(["flite", "-voice", "rms", "-t", text])

# Example usage


# Set master volume to 75%
call(["amixer", "-D", "pulse", "sset", "Master", "50%"])
#call(["pactl", "set-sink-volume", "@DEFAULT_SINK@", "+25%"])


# Toggle mute on master
#call(["amixer", "-D", "pulse", "sset", "Master", "toggle"])

speak_flite("This is a spectrogram of my voice coming from the speaker.")
