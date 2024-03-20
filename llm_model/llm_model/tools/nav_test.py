#!/usr/bin/env python3

import json
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation import (
    TurtleBot4Directions,
    Turtlebot4Navigator,
)


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber_test")
        self.get_logger().info("Initialising pose subscriber test node...")
        # self.navigator = BasicNavigator()
        self.turtlebot4_navigator = Turtlebot4Navigator()
        self.navigation_task = None
        # self.navigator.waitUntilNav2Active()
        self.turtlebot4_navigator.waitUntilNav2Active(localizer="bt_navigator")
        self.save_state(None)
        if not self.turtlebot4_navigator.getDockedStatus():
            self.turtlebot4_navigator.dock()
        initial_pose = self.turtlebot4_navigator.getPoseStamped(
            [-8.71, -2.49], TurtleBot4Directions.NORTH
        )
        # self.navigator.setInitialPose(initial_pose)
        self.turtlebot4_navigator.setInitialPose(initial_pose)
        self.retrieve_commands()
        # self.pose_subscription = self.create_subscription(
        #     String, "/pose_listener", self.listener_callback, 10
        # )
        self.vision_publisher = self.create_publisher(String, "/nav_vision_module", 10)

    def retrieve_commands(self):
        self.goals = {
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
            "sleep": {
                "position": [-0.813, 1.171],
                "orientation": 0,
                "description": "Dock",
            },
        }

    def listener_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        command = msg.data
        if command == "stop":
            self.stop_task()
        elif command in self.goals:
            self.navigate_to(command)
        else:
            self.get_logger().warn(
                f"Invalid command: {command}. Valid commands are: {list(self.goals.keys())}."
            )

    def navigate_to(self, destination):
        if destination == "sleep" and not self.turtlebot4_navigator.getDockedStatus():
            self.turtlebot4_navigator.dock()
            return
        position = self.goals[destination]["position"]
        orientation = TurtleBot4Directions(self.goals[destination]["orientation"])
        goal_pose = self.turtlebot4_navigator.getPoseStamped(position, orientation)
        self.stop_task()
        self.navigation_task = self.turtlebot_navigator.goToPose(goal_pose)

    def save_state(self, state):
        self.state_path = "state.json"
        with open(self.state_path, "w") as file:
            json.dump({"state": state}, file)

    def stop_task(self):
        if self.navigation_task is not None:
            self.navigator.cancelTask()
            self.navigation_task = None


if __name__ == "__main__":
    rclpy.init()
    node = NavigatorNode()
    node.navigate_to("bakery")
    rclpy.spin(node)
    rclpy.shutdown()
