import json
from typing import Optional, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool
from .turtlebot4_navigation import Turtlebot4Navigator, TurtleBot4Directions
from rclpy.publisher import Publisher
from std_msgs.msg import String
waypoints = "corner, dock, undock"
# with open("waypoints.json") as f:
#     waypoints = json.load(f)

class NavigateInput(BaseModel):
    waypoint: str = Field(..., description=f"Waypoints to navigate to: {waypoints}. Choose one")


class NavigateTool(BaseTool):
    name = "navigate_to_waypoint"
    description = ("Navigate to a waypoint, useful for moving the robot to a specific location")
    args_schema: Type[BaseModel] = NavigateInput
    navigator: Publisher = None

    def __init__(self, navigator):
        super().__init__()
        self.navigator = navigator

    def _run(
        self,
        waypoint: str,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ):
        """
        Publishes a string message to a topic.
        """
        msg = String()
        msg.data = waypoint
        self.navigator.publish(msg)



    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        pass

if __name__ == "__main__":
    tool = NavigateTool(Turtlebot4Navigator())
    tool._run("corner")