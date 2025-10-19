from dotenv import load_dotenv
import openai
import requests
import subprocess
import io
import os
import time



#this loads the api key from *.env file so it is not stored locally
load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

print("KEY: ", OPENAI_API_KEY)

# Function to stream TTS audio in near real-time using FFmpeg
def stream_tts(text):
    url = "https://api.openai.com/v1/audio/speech"
    headers = {
        "Authorization": f"Bearer {OPENAI_API_KEY}",
        "Content-Type": "application/json"
    }
    data = {
        "model": "tts-1",  # or "tts-1-hd"
        "input": text,
        "voice": "nova"   # Choose from "alloy", "echo", "fable", "onyx", "nova", "shimmer"
    }

    response = requests.post(url, headers=headers, json=data, stream=True)

    if response.status_code == 200:
        process = subprocess.Popen(
            ["ffplay", "-nodisp", "-autoexit", "-"],  # Play audio without display
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Stream and play chunks as they arrive
        for chunk in response.iter_content(chunk_size=4096):  
            if chunk:
                process.stdin.write(chunk)
                process.stdin.flush()

        process.stdin.close()
        process.wait()  # Wait for playback to finish

    else:
        print("Error:", response.json())

# Example usage
while True:
    text = input("What to say? ")
    start = time.time()
    stream_tts(text)
    print(time.time() - start, "seconds.")
