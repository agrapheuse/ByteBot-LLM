import asyncio
import json
import asyncio
import json
import os
import subprocess

import pyaudio
import websockets
import websockets


class SpeechRecognition:
    """
    This class is responsible for recognizing speech from the microphone and sending it to the Deepgram API.
    """

    def __init__(
        self,
        deepgram_api_key="161f0950caa93401ba308a0546d658acad90dd67",
        input_device_index=3,
        log_path="/tmp/voice.txt",
        activation_function=None,
    ):
        self.deepgram_api_key = deepgram_api_key
        self.input_device_index = input_device_index
        self.log_path = log_path
        self.audio_queue = asyncio.Queue()
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024
        self.is_running = False
        self.activation_function = activation_function
        if log_path:
            with open(log_path, "w") as f:
                f.write("")

    def callback(self, input_data, frame_count, time_info, status_flags):
        self.audio_queue.put_nowait(input_data)
        return (input_data, pyaudio.paContinue)

    async def microphone(self):
        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
            stream_callback=self.callback,
            input_device_index=self.input_device_index,
        )
        stream.start_stream()
        while stream.is_active():
            await asyncio.sleep(0.1)
        stream.stop_stream()
        stream.close()

    def meaningfully_split(self, transcribed):
        if "hello adam" in transcribed:
            return transcribed.lower().split("hello adam")
        elif "hey adam" in transcribed:
            return transcribed.lower().split("hey adam")

    async def process(self):
        extra_headers = {"Authorization": "token " + self.deepgram_api_key}
        async with websockets.connect(
            "wss://api.deepgram.com/v1/listen?encoding=linear16&sample_rate=16000&channels=1&model=nova-2",
            extra_headers=extra_headers,
        ) as ws:
            await asyncio.gather(self.sender(ws), self.receiver(ws))

    async def sender(self, ws):
        try:
            while True:
                data = await asyncio.wait_for(self.audio_queue.get(), timeout=5.0)
                await ws.send(data if data else b"")
        except Exception as e:
            print("Error while sending: ", e)
            raise

    async def receiver(self, ws):
        async for msg in ws:
            try:
                msg = json.loads(msg)
                transcript = msg["channel"]["alternatives"][0]["transcript"]
                if transcript:
                    to_save = transcript + " "
                else:
                    to_save = "."

                if self.log_path:
                    with open(self.log_path, "a") as f:
                        f.write(to_save)
            except Exception as e:
                print("Error while receiving: ", e)

    def act_on_activation_word(self, transcribed):
        adam_detected = False
        if self.keyword_detected(transcribed):
            adam_detected = True
            temp_content = self.meaningfully_split(transcribed)
            temp_content = temp_content[-1] if temp_content else ""
        if adam_detected:
            return temp_content
        return transcribed

    async def run(self):
        if not self.is_running:
            self.is_running = True
            await asyncio.gather(self.microphone(), self.process())


if __name__ == "__main__":
    recognizer = SpeechRecognition()
    try:
        asyncio.run(recognizer.run())
    except Exception as e:
        print("Error: ", e)
