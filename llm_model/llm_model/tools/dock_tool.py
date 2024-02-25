from typing import Optional

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.tools import BaseTool
from rclpy.publisher import Publisher


class DockTool(BaseTool):
    name = "dock"
    description = "Dock the robot to a charging station, useful for recharging the robot or just for resting it"
    publisher: Publisher = None

    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes a string message to a topic.
        """
        self.publisher.publish({})
        self.get_logger().info("Docking the robot")
        return "Docking the robot"

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")
