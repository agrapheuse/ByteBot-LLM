import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .tools import DockTool, SpeechTool, SteerTool, Turtlebot4Navigator, DanceTool, NavigateTool, VoiceCloningTool
from .agent import Agent
import time

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
            String, "/llm_input_audio_to_text", self.llm_callback, 10
        )

        # Publishers
        self.plan_publisher = self.create_publisher(String, "/llm_plan", 10)
        self.tool_publisher = self.create_publisher(String, "/llm_tool", 10)
        # Plan topic, meant for use with the frontend
        self.plan_subscriber = self.create_subscription(
            String, "/llm_plan", self.plan_callback, 10
        )

        # Tool topic, when you want to use tools
        self.tool_subscriber = self.create_subscription(
            String, "/llm_tool", self.tool_callback, 10
        )

        # LLM response type publisher
        self.llm_response_type_publisher = self.create_publisher(
            String, "/llm_response_type", 0
        )

        # LLM feedback for user publisher
        self.llm_feedback_publisher = self.create_publisher(
            String, "/llm_feedback_to_user", 0
        )

        # Navigation publisher
        self.navigation_publisher = self.create_publisher(
            String, "/pose_listener", 0
        )

        self.steer_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.navigator = Turtlebot4Navigator()
        # Function name
        self.function_name = "null"
        self.agent = Agent(self._get_tools(), self.get_logger())
        # Initialization ready
        self.publish_string("llm_model ready", self.initialization_publisher)

    def llm_callback(self, msg):
        """
        Callback function for the LLM input listener.
        This function is called when a message is received on the LLM input topic.
        """
        self.get_logger().info(f"BRAIN ==== Received message: {msg.data}")
        self.agent.act(msg.data)
        self.get_logger().info(f"Spoke: '{msg.data}'")

    def plan_callback(self, msg):
        """
        Callback function for the plan listener.
        This function is called when a message is received on the plan topic.
        """
        self.get_logger().info(f"Received plan: {msg.data}")
        self.agent.act_from_plan(msg.data)

    def tool_callback(self, msg):
        """
        Callback function for the tool listener.
        This function is called when a message is received on the tool topic.
        """
        self.get_logger().info(f"Received tool: {msg.data}")
        self.agent.act_from_tool(msg.data)

    def publish_string(self, data, publisher):
        """
        Publishes a string message to a topic.
        """
        msg = String()
        msg.data = data
        publisher.publish(msg)
        self.get_logger().info(f"Published message: {msg.data}")

    def _get_tools(self):
        """
        Returns the tools that the agent can use.
        """
        steer_tool = SteerTool(self.steer_publisher)
        speech_tool = SpeechTool(
            self.llm_feedback_publisher
        )
        dock_tool = DockTool(self.navigator)  # USE NOAH'S DOCK TOOL, ONLY WORKS WITH NAV TOPIC
        navigate_tool = NavigateTool(self.navigation_publisher)
        dance_tool = DanceTool()
        voice_cloning_tool = VoiceCloningTool()
        from langchain.tools import HumanInputRun
        human_input = HumanInputRun(input_func=self.get_human_input)

        tools = [steer_tool, speech_tool, dance_tool, dock_tool, voice_cloning_tool, navigate_tool, human_input]
        return tools

    def get_human_input(self, secs=5, timeout=30):
        """
        Get human input from the user, with an improved performance approach.
        """
        TRANSCRIPTION_LOG = os.path.join(
            os.expanduser("~"), "bytebot", "knowledge", "voice.txt"
        )
        TRANSCRIPTION_MARKER = "===TRANSCRIPTION==="
        start_time = time.time()

        # Initialize transcription log
        with open(TRANSCRIPTION_LOG, "w") as f:
            f.write(TRANSCRIPTION_MARKER + "\n")

        # Loop until input is captured or timeout
        while True:
            # Check for timeout
            if time.time() - start_time > timeout:
                print("Timeout waiting for human input.")
                return None

            time.sleep(secs)
            try:
                with open(TRANSCRIPTION_LOG, "r") as f:
                    transcription = f.read()
                    if TRANSCRIPTION_MARKER in transcription:
                        content_after_marker = transcription.split(
                            TRANSCRIPTION_MARKER
                        )[1].strip()
                        if content_after_marker.endswith(".."):
                            return content_after_marker[:-2].strip()
            except FileNotFoundError:
                print(f"File {TRANSCRIPTION_LOG} not found.")
                break

        return None


def main(args=None):
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
