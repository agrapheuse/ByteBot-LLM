import os
import sys

import rclpy
from langchain import hub
from langchain.agents import create_openai_functions_agent
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_agent_executor
from rclpy.node import Node
from std_msgs.msg import String
from .agent import Agent

from geometry_msgs.msg import Twist
from .tools import get_tools, read_chat_history, SteerTool, SpeechTool

sys.path.append("..")
print(os.getcwd())


class Brain(Node):
    """
    Class for the Brain node. This is where the LLM logic is implemented.
    """

    def __init__(self):
        super().__init__("ChatGPT_node")
        # Initialization publisher
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization_state", 0
        )

        # LLM state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)

        # LLM state listener
        # self.llm_state_subscriber = self.create_subscription(
        #     String, "/llm_state", self.state_listener_callback, 0
        # )
        # LLM input listener
        self.llm_input_subscriber = self.create_subscription(
            String, "/llm_input_audio_to_text", self.llm_callback, 0
        )
        # LLM response type publisher
        self.llm_response_type_publisher = self.create_publisher(
            String, "/llm_response_type", 0
        )

        # LLM feedback for user publisher
        self.llm_feedback_publisher = self.create_publisher(
            String, "/llm_feedback_to_user", 0
        )
        # Function name
        self.function_name = "null"
        # Initialization ready
        self.publish_string("llm_model_processing", self.initialization_publisher)

    def llm_callback(self, msg):
        """
        Callback function for the LLM input listener.
        This function is called when a message is received on the LLM input topic.
        """
        self.publish_string("listening", self.llm_state_publisher)
        self.publish_string("processing", self.llm_state_publisher)
        steer_tool = SteerTool(self.create_publisher(Twist, "/cmd_vel", 10))
        speech_tool = SpeechTool(self.create_publisher(String, "/llm_input_text_to_audio", 0))
        tools = [steer_tool, speech_tool]
        agent = Agent(tools, self.get_logger())
        agent.act(msg.data)

    def publish_string(self, data, publisher):
        """
        Publishes a string message to a topic.
        """
        msg = String()
        msg.data = data
        publisher.publish(msg)
        self.get_logger().info(f"Published message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
