import asyncio
from dotenv import load_dotenv
from openai import AsyncOpenAI
from openai.helpers import LocalAudioPlayer
import os
import time

#this loads the api key from *.env file so it is not stored locally
load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

openai = AsyncOpenAI(api_key=OPENAI_API_KEY)
start = time.time()
async def main() -> None:
    async with openai.audio.speech.with_streaming_response.create(
        model="gpt-4o-mini-tts",
        voice="sage",
        input="Hey Johnny, how are you today?  How is Charlie?",
        instructions="Like you are wired on drugs and you can't control your tone.",
        #instructions="Speak in a cheerful and positive tone.",
        response_format="pcm",
    ) as response:
        await LocalAudioPlayer().play(response)

if __name__ == "__main__":
    asyncio.run(main())
print(f"Time: {time.time()-start}")