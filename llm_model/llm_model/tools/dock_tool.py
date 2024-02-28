from typing import Optional, Union, Dict, Tuple, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.tools import BaseTool
from rclpy.publisher import Publisher
from langchain.pydantic_v1 import BaseModel, Field


class DockInput(BaseModel):
    to_dock: bool = Field(..., description="True to dock, False to undock")


class DockTool(BaseTool):
    name = "dock"
    description = "Dock or undock the robot to a charging station, useful"
    publisher_to_dock: Publisher = None
    publisher_to_undock: Publisher = None
    args_schema: Type[BaseModel] = DockInput

    def __init__(self, publisher_to_dock: Publisher, publisher_to_undock: Publisher):
        super().__init__()
        self.publisher_to_dock = publisher_to_dock
        self.publisher_to_undock = publisher_to_undock

    def _run(
        self,
        to_dock: bool,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes a string message to a topic.
        """
        if to_dock:
            self.publisher_to_dock.publish({})
            self.get_logger().info("Docking the robot")
            return "Docking the robot"
        else:
            self.publisher_to_undock.publish({})
            self.get_logger().info("Undocking the robot")
            return "Unocking the robot"

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")

    def _to_args_and_kwargs(self, tool_input: Union[str, Dict]) -> Tuple[Tuple, Dict]:
        return (), {}
