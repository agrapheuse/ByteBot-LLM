import asyncio
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .voice import SpeechRecognition

from .voice import SpeechRecognition


class AudioInput(Node):
    def __init__(self):
        super().__init__("llm_audio_input")
        self.tmp_audio_file = "/tmp/user_audio_input.flac"
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization_state", 0
        )
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)
        self.llm_state_subscriber = self.create_subscription(
            String, "/llm_state", self.state_listener_callback, 0
        )
        self.audio_to_text_publisher = self.create_publisher(
            String, "/llm_input_audio_to_text", 0
        )
        self.sr = SpeechRecognition(log_path="/tmp/voice.txt")
        self.timer = self.create_timer(1, self.action_function_listening)
        self.publish_string("llm_audio_input", self.initialization_publisher)
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()

    def state_listener_callback(self, msg):
        self.get_logger().info(f"STATE: {msg.data}")
        if msg.data == "listening":
            asyncio.run_coroutine_threadsafe(self.sr.run(), self.loop)

    async def action_function_listening(self):
        with open("/tmp/voice.txt", "r") as f:
            transcribed = f.read()
        self.get_logger().info(f"Transcribed: {transcribed}")
        if self.keyword_detected(transcribed) and transcribed[-2:] == "..":
            self.get_logger().info("Keyword detected!")
            temp_content = self.sr.meaningfully_split(transcribed)
            temp_content = temp_content[-1] if temp_content else ""
            # Strip all the dots
            temp_content = temp_content.replace(".", "")
            self.get_logger().info(f"TEMP CONTENT: {temp_content}")
            self.publish_string(temp_content, self.audio_to_text_publisher)
            with open("/tmp/voice.txt", "w") as f:
                f.write("")

    def keyword_detected(self, transcribed):
        keywords = ["hey adam", "hello adam"]
        return any(keyword in transcribed.lower() for keyword in keywords)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    audio_input = AudioInput()
    rclpy.spin(audio_input)
    audio_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
