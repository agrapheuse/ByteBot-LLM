#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
#
# Node test Method:
# ros2 run llm_output llm_audio_output
# ros2 topic pub /llm_feedback_to_user std_msgs/msg/String "data: 'Hello, welcome to ROS-LLM'" -1
#
# Author: Herman Ye @Auromix

# Other libraries
import datetime
import json
import requests
import time

import os

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from elevenlabs import set_api_key, generate, play

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()


class AudioOutput(Node):
    def __init__(self):
        super().__init__("audio_output")

        # Initialization publisher
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization_state", 0
        )

        # eleven labs
        set_api_key(os.environ["ELEVENLABS_API_KEY"])

        # LLM state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)

        # Feedback for user listener
        self.feed_back_for_user_subscriber = self.create_subscription(
            String, "/llm_feedback_to_user", self.feedback_for_user_callback, 10
        )

        # Initialization ready
        self.publish_string("output ready", self.initialization_publisher)

    def feedback_for_user_callback(self, msg):
        self.get_logger().info(f"Received text: '{msg.data}'")
        # Log and publish state updates
        self.get_logger().info("Finished TTS playback.")
        self.publish_string("feedback finished", self.llm_state_publisher)
        self.publish_string("listening", self.llm_state_publisher)
        if msg.data:
            self.get_logger().info(f"MSG DATA FOUND - PLAYING  {msg.data}")
            audio = generate(
                text=msg.data,
                voice="George - royal and elegant",
            )
            # Save to tmp
            with open("/tmp/speech_output.mp3", "wb") as file:
                file.write(audio)
            play(audio, use_ffmpeg=True)
            self.publish_string("feedback finished", self.llm_state_publisher)
            self.publish_string("listening", self.llm_state_publisher)

        # # Save the audio output to a file
        # output_file_path = "/tmp/speech_output.mp3"
        # with open(output_file_path, "wb") as file:
        #     file.write(response["AudioStream"].read())
        # # Play the audio output
        # os.system("mpv" + " " + output_file_path)
        # self.get_logger().info("Finished Polly playing.")
        # self.publish_string("feedback finished", self.llm_state_publisher)
        # self.publish_string("listening", self.llm_state_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)

    audio_output = AudioOutput()

    rclpy.spin(audio_output)

    audio_output.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
