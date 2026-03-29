import subprocess
from subprocess import call

def speak_flite(text):
    """Speaks the given text using flite."""
    subprocess.run(["flite", "-voice", "rms", "-t", text])
    #available voices: kal, awb_time, kal16, awb, rms, slt

# Example usage


# Set master volume to 75%
call(["amixer", "-D", "pulse", "sset", "Master", "100%"])
#call(["pactl", "set-sink-volume", "@DEFAULT_SINK@", "+25%"])


# Toggle mute on master
#call(["amixer", "-D", "pulse", "sset", "Master", "toggle"])

speak_flite("Hello Morgan.")
