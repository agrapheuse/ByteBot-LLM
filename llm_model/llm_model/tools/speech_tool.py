import datetime
import json
import os
import subprocess
from typing import Iterator, Optional, Type

# Audio recording related
from elevenlabs import Voice, generate
from elevenlabs.client import ElevenLabs
from gtts import gTTS
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, tool

# Global Initialization
from llm_config.user_config import UserConfig
from rclpy.publisher import Publisher

config = UserConfig()

CHEAP = True
flag_file_path = "/tmp/voice_cloning_flag.txt"
api_key = "c099a9d746f0d3f8ad573e47223b85a8"
client = ElevenLabs(api_key=api_key)


class SpeechInput(BaseModel):
    text: str = Field(..., description="The text to be converted to audio")


class SpeechTool(BaseTool):
    name = "speech"
    description = "useful for converting text to audio and saying something to the user, use this often to give feedback to the user"
    args_schema: Type[BaseModel] = SpeechInput
    publisher: Publisher = None
    device_index: int = 7

    def __init__(self, publisher, device_index=1):
        super().__init__()
        self.publisher = publisher

    def _run(
        self,
        text: str,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes a string message to a topic.
        """
        # This approach is really slow, we don't need to publish to a topic
        # msg = String()
        # msg = text
        # self.publisher.publish(msg)
        self.feedback_for_user_callback(text)

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")

    def feedback_for_user_callback(self, msg):
        current_time = datetime.datetime.now()
        try:
            with open("/tmp/feedback_log.json", "r") as file:
                feedback_log = json.load(file)
        except FileNotFoundError:
            feedback_log = {}

        if msg in feedback_log:
            last_spoken_time = datetime.datetime.fromisoformat(feedback_log[msg])
            if (current_time - last_spoken_time).total_seconds() < 300:
                return
        feedback_log[msg] = current_time.isoformat()

        with open("/tmp/feedback_log.json", "w") as file:
            json.dump(feedback_log, file)
        os.system("killall mpv")
        if msg:
            if CHEAP:
                self.play_generic_tts(msg)
            else:
                self.play_tts(msg)

    def play_generic_tts(self, msg):
        tts = gTTS(text=msg, lang="en")
        tts_file = "/tmp/speech_output.mp3"
        tts.save(tts_file)
        os.system(f"mpv {tts_file}")

    def play_tts(self, msg):
        # if flag_file_path exists, use the cloned voice
        if os.path.exists(flag_file_path):
            with open(flag_file_path, "r") as f:
                voice_id = f.read()
            audio = generate(
                text="....... " + msg,
                voice=Voice(
                    api_key=api_key,
                    voice_id=voice_id,
                    settings=client.voices.get_settings(voice_id),
                ),
                stream=True,
            )
        # otherwise, use the default voice
        else:
            audio = generate(
                api_key=api_key,
                text="....... " + msg,
                voice=Voice(
                    api_key=api_key,
                    voice_id="DxwyQfgZrGyVfTtqkF2O",
                    settings=client.voices.get_settings("DxwyQfgZrGyVfTtqkF2O"),
                ),
                stream=True,
            )

        self._stream(audio)

    def _stream(self, audio_stream: Iterator[bytes]) -> bytes:
        mpv_command = [
            "mpv",
            "--no-cache",
            "--no-terminal",
            "--",
            "fd://0",
            "--audio-device=alsa/hw:1,0",
        ]
        mpv_process = subprocess.Popen(
            mpv_command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        audio = b""

        for chunk in audio_stream:
            if chunk is not None:
                mpv_process.stdin.write(chunk)  # type: ignore
                mpv_process.stdin.flush()  # type: ignore
                audio += chunk
        if mpv_process.stdin:
            mpv_process.stdin.close()
        mpv_process.wait()

        return audio


if __name__ == "__main__":
    tool = SpeechTool(None)
    tool._run("Hello, I am Adam")
