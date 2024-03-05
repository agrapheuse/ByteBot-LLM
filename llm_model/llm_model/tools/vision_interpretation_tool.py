from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, StructuredTool, tool
from typing import Type, Optional
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from geometry_msgs.msg import Twist
from rclpy.subscription import Subscription
from std_msgs.msg import String
import datetime
import json
import requests
import time

import os


# Global Initialization
from llm_config.user_config import UserConfig

import base64
from dotenv import load_dotenv
load_dotenv()
api_key = os.getenv("OPENAI_API_KEY")
config = UserConfig()


class VisionTool(BaseTool):
    name = "vision"
    description = "Useful for interpreting what you're seeing right now. Use this to understand the world around you."
    subscription: Subscription = None

    def __init__(self, subscription):
        super().__init__()
        self.subscription = subscription

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes a string message to a topic.
        """
        base64_image = self.encode_image(self.get_image())

        headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
        }

        payload = {
        "model": "gpt-4-vision-preview",
        "messages": [
            {
            "role": "user",
            "content": [
                {
                "type": "text",
                "text": "What's in this image?"
                },
                {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{base64_image}"
                }
                }
            ]
            }
        ],
        "max_tokens": 300
        }

        response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

        print(response.json())
        return response.json()["choices"][0]["message"]["content"][0]["text"]


    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")

    def encode_image(self, image):
        return base64.b64encode(image).decode('utf-8')
    
    def get_image(self):
        self.subscription