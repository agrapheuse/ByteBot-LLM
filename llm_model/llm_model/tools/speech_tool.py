from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, StructuredTool, tool
from typing import Type, Optional
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from std_msgs.msg import String
import datetime
import json
import requests
import time

import os

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from elevenlabs import set_api_key, generate, play

# Global Initialization
from llm_config.user_config import UserConfig

from gtts import gTTS

config = UserConfig()

CHEAP = True
class SpeechInput(BaseModel):
    text: str = Field(..., description="The text to be converted to audio")


class SpeechTool(BaseTool):
    name = "speech"
    description = (
        "useful for converting text to audio and saying something to the user, use this often to give feedback to the user"
    )
    args_schema: Type[BaseModel] = SpeechInput
    publisher: Publisher = None

    def __init__(self, publisher):
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
        # Log and publish state updates
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
        os.system(f"mpv --audio-device=alsa/hw:1,0 {tts_file}")

    def play_tts(self, msg):
        audio = generate(
            text="....... " + msg,
            voice="George - royal and elegant",
        )
        # Save to tmp
        with open("/tmp/speech_output.mp3", "wb") as file:
            file.write(audio)
        os.system("mpv  --audio-device=alsa/hw:1,0 /tmp/speech_output.mp3")