from typing import Optional, Type

from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from langchain.tools import BaseTool
from .turtlebot4_navigation import Turtlebot4Navigator


class DockInput(BaseModel):
    to_dock: bool = Field(..., description="True to dock, False to undock")


class DockTool(BaseTool):
    name = "dock"
    description = """Dock or undock the robot from a charging station, useful for recharging the robot or enabling
Can be synonimous with 'charge' or 'recharge', 'wake up' or 'sleep'"""
    args_schema: Type[BaseModel] = DockInput
    navigator: Turtlebot4Navigator = None

    def __init__(self, navigator):
        super().__init__()
        self.navigator = navigator

    def _run(
        self,
        to_dock: bool,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ):
        """
        Publishes a string message to a topic.
        """
        if to_dock:
            self.navigator.dock()
        else:
            self.navigator.undock()

        return "Docking or undocking the robot"

    async def _arun(
        self, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        pass


if __name__ == "__main__":
    from langchain.tools import run_tool
    tool = DockTool(Turtlebot4Navigator())
    run_tool(tool, {"to_dock": True})