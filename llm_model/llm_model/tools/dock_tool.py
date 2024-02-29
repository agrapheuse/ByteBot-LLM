from typing import Optional, Union, Dict, Tuple, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.tools import BaseTool
from rclpy.publisher import Publisher
from langchain.pydantic_v1 import BaseModel, Field
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock


class DockInput(BaseModel):
    to_dock: int = Field(..., description="0 to undock, 1 to dock")


class DockTool(BaseTool):
    name = "dock"
    description = "Dock or undock the robot from a charging station, useful for recharging the robot or enabling movement"
    publisher_to_dock: ActionClient = None
    publisher_to_undock: ActionClient = None
    args_schema: Type[BaseModel] = DockInput

    def __init__(self, publisher_to_dock: ActionClient, publisher_to_undock: ActionClient):
        super().__init__()
        self.publisher_to_dock = publisher_to_dock
        self.publisher_to_undock = publisher_to_undock

    def _run(
        self,
        to_dock: int,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes a string message to a topic.
        """
        if to_dock == 1:
            dock_goal = Dock.Goal()
            self.publisher_to_dock.send_goal_async(dock_goal)
            return "Docking the robot"
        else:
            undock_goal = Undock.Goal()
            self.publisher_to_undock.send_goal_async(undock_goal)
            return "Unocking the robot"

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")