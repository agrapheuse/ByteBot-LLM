# mpv  --audio-device=alsa/hw:1,0 https://www.youtube.com/watch?v=eSBybJGZoCU --no-video
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool, StructuredTool, tool
from typing import Type, Optional
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)

import os



class SongInput(BaseModel):
    text: str = Field(..., description="The song name to be queried and played")


class SongTool(BaseTool):
    name = "song_player"
    description = (
        "useful for playing sounds, music from youtube. Use this to play music or sounds"
    )
    args_schema: Type[BaseModel] = SongInput

    def __init__(self):
        super().__init__()

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
        link = self.get_song_link(text)
        os.system(f"mpv --audio-device=alsa/hw:1,0 {link} --no-video")

    async def _arun(
            self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")

    def get_song_link(self, song_name):
        return "https://www.youtube.com/watch?v=eSBybJGZoCU"


if __name__ == "__main__":
    tool = SongTool()
    tool._run("hello")