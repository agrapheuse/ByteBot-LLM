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
        msg = String()
        msg.data = text
        self.publisher.publish(msg)

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")
