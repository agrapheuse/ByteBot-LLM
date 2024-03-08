import asyncio
import json

import pyaudio
import websockets
import os
import subprocess

DEEPGRAM_API_KEY = "161f0950caa93401ba308a0546d658acad90dd67"

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024

LOG_PATH = os.path.join(os.path.expanduser("~"), "bytebot", "knowledge", "voice.txt")
with open(LOG_PATH, "w") as f:
    f.write("")

audio_queue = asyncio.Queue()


def callback(input_data, frame_count, time_info, status_flags):
    audio_queue.put_nowait(input_data)

    return (input_data, pyaudio.paContinue)


async def microphone():
    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK,
        stream_callback=callback,
        input_device_index=1,
    )

    stream.start_stream()

    while stream.is_active():
        await asyncio.sleep(0.1)

    stream.stop_stream()
    stream.close()


def keyword_detected(transcribed):
    keywords = ['hey adam', 'hello adam']
    if any(keyword in transcribed.lower() for keyword in keywords):
        return True
    return False

def meaningfully_split(transcribed):
    if "hello adam" in transcribed:
        return transcribed.lower().split("hello adam")
    elif "hey adam" in transcribed:
        return transcribed.lower().split("hey adam")


async def process():
    extra_headers = {"Authorization": "token " + DEEPGRAM_API_KEY}
    async with websockets.connect(
        "wss://api.deepgram.com/v1/listen?encoding=linear16&sample_rate=16000&channels=1&model=nova-2-meeting",
        extra_headers=extra_headers,
    ) as ws:

        async def sender(ws):  # sends audio to websocket
            try:
                while True:
                    data = await asyncio.wait_for(audio_queue.get(), timeout=5.0)

                    await ws.send(data if data else b"")
            except Exception as e:
                print("Error while sending: ", +str(e))
                raise

        def act_on_activation_word(transcribed):
            adam_detected = False
            if keyword_detected(transcribed):
                adam_detected = True
                temp_content = meaningfully_split(transcribed)
                print(f"TEMP CONTENT: {temp_content}")
                temp_content = temp_content[-1]
            if adam_detected and temp_content[-1] == ".":
                print(f"SENDING MESSAGE: {temp_content}")
                # escape all single quotes
                temp_content = temp_content.replace("'", "")
                command = f"""ros2 topic pub --once /llm_input_audio_to_text std_msgs/msg/String \"data: '{temp_content}'\""""
                print(command)
                for _ in range(5):
                    subprocess.Popen(["bash", "-c", command])
                print("CLEARING TRANSCRIBED DATA")
                subprocess.Popen(["bash", "-c", "mpv  --audio-device=alsa/hw:1,0 ~/bytebot/ping.mp3"])
                return ""
            return transcribed

        async def receiver(ws):
            transcribed_data = ""
            async for msg in ws:
                try:
                    msg = json.loads(msg)
                    transcript = msg["channel"]["alternatives"][0]["transcript"]
                    if transcript:
                        transcribed_data += transcript + " "
                        print(f"Transcript = {transcript}")
                    else:
                        transcribed_data += "."
                    transcribed_data = act_on_activation_word(transcribed_data)
                    with open(LOG_PATH, "a") as f:
                        f.write(transcribed_data)
                except Exception as e:
                    print("Error while receiving: ", e)

        await asyncio.gather(sender(ws), receiver(ws))


async def run():
    await asyncio.gather(microphone(), process())


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except Exception as e:
        print("Error in main: ", e)
        asyncio.run(run())
