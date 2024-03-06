from typing import Optional, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool
from .turtlebot4_navigation import Turtlebot4Navigator




class AwaitInputTool(BaseTool):
    name = "await_input"
    description = (
      "Await input from the user, useful for waiting for user input before executing the next command"
    )

    def __init__(self):
        super().__init__()

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ):
        """
        Save data to a file.
        """
        user_input = "test"

        return user_input

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        pass
