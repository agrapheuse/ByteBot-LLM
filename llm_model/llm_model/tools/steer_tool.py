from typing import Optional, Type

from geometry_msgs.msg import Twist
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from langchain.pydantic_v1 import BaseModel, Field
from rclpy.publisher import Publisher
from langchain.tools import BaseTool


class SteerInput(BaseModel):
    linear_x: float = Field(..., description="The linear velocity along the x-axis")
    linear_y: float = Field(..., description="The linear velocity along the y-axis")
    linear_z: float = Field(..., description="The linear velocity along the z-axis")
    angular_x: float = Field(..., description="The angular velocity around the x-axis")
    angular_y: float = Field(..., description="The angular velocity around the y-axis")
    angular_z: float = Field(..., description="The angular velocity around the z-axis")


class SteerTool(BaseTool):
    name = "steer_robot"
    description = (
        "useful for steering the robot - moving forward, backward, left, right. the higher the value, the faster the robot moves. use this often to control the robot's movement."
    )
    args_schema: Type[BaseModel] = SteerInput
    publisher: Publisher = None

    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher

    def _run(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_x: float = 0.0,
        angular_y: float = 0.0,
        angular_z: float = 0.0,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ) -> str:
        """
        Publishes cmd_vel message to control the movement of turtlesim or a similar entity.

        Parameters:
        - steer_input: Input for steering control.
        - linear_x: Linear velocity in the X direction.
        - linear_y: Linear velocity in the Y direction.
        - linear_z: Linear velocity in the Z direction.
        - angular_x: Angular velocity around the X axis.
        - angular_y: Angular velocity around the Y axis.
        - angular_z: Angular velocity around the Z axis.
        - run_manager: Optional callback manager for tool run.

        Returns:
        A string representation of the Twist message published.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z

        self.publisher.publish(twist_msg)
        return f"Published cmd_vel message: {twist_msg}"

    async def _arun(
        self, query: str, run_manager: Optional[AsyncCallbackManagerForToolRun] = None
    ) -> str:
        """Use the tool asynchronously."""
        raise NotImplementedError("custom_search does not support async")
