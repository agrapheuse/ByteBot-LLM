import os
from typing import Optional, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool
from rclpy.publisher import Publisher
from std_msgs.msg import String

from .turtlebot4_navigation import TurtleBot4Directions, Turtlebot4Navigator

USER_DIR = os.path.expanduser("~")
# with open(os.path.join(USER_DIR, "bytebot", "commands.json"), "r") as file:
#     commands = json.load(file)
commands = {
    "stage": {
        "position": [-2.415, 4.013],
        "orientation": 180,
        "description": "Entrance of the room",
    },
    "bakery": {
        "position": [-2.492, 10.745],
        "orientation": 90,
        "description": "bread corner",
    },
    "cheese section": {
        "position": [-3.746, 0.947],
        "orientation": 90,
        "description": "Cisco storage room",
    },
    "sleep": {"position": [-0.813, 1.171], "orientation": 0, "description": "Dock"},
    "wake up": {"position": [-0.813, 1.171], "orientation": 0, "description": "Undock"},
}

parsed_waypoints_str = "\n".join(
    [f"{name} : {waypoint['description']}" for name, waypoint in commands.items()]
)


class NavigateInput(BaseModel):
    waypoint: str = Field(
        ...,
        description=f"Waypoints to navigate to: {parsed_waypoints_str}. Choose one, only the name",
    )


class NavigateTool(BaseTool):
    name = "navigate_to_waypoint"
    description = (
        "Navigate to a waypoint, useful for moving the robot to a specific location"
    )
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
        print(f"Published message: {msg.data}")
        self.navigator.publish(msg)

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        pass


if __name__ == "__main__":
    tool = NavigateTool(Turtlebot4Navigator())
    tool._run("corner")
