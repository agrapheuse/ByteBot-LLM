#!/usr/bin/env python3

import json

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.get_logger().info("Initialising pose subscriber node...")
        self.navigator = BasicNavigator()
        self.turtlebot4_navigator = TurtleBot4Navigator()

        self.navigation_task = None  # Variable to track the current navigation task

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()
        self.save_state("None")
        # Start on dock
        if not self.turtlebot4_navigator.getDockedStatus():
            self.turtlebot4_navigator.info("Docking before initialising pose")
            self.turtlebot4_navigator.dock()

        # Set initial pose
        initial_pose = self.turtlebot4_navigator.getPoseStamped(
            [-8.71, -2.49], TurtleBot4Directions.NORTH
        )
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info("Retrieving commands...")
        self.retrieve_commands()
        self.get_logger().info("Retrieved goals:")
        for i in self.goals:
            self.get_logger().info(i)
        self.get_logger().info("Commands retrieved")

        self.pose_subscription = self.create_subscription(
            String, "/pose_listener", self.listener_callback, 10
        )
        self.get_logger().info(
            'listening to /pose_listener topic, publish "list" to see all commands.'
        )
        self.pose_subscription

        self.vision_publisher = self.create_publisher(String, "/nav_vision_module", 10)

    def retrieve_commands(self):
        data = {
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
        return data

    def listener_callback(self, msg):
        self.get_logger().info("Received command: " + msg.data)
        self.retrieve_commands()
        command = msg.data
        if command == "stop":
            msg = String()
            msg.data = "None"
            self.vision_publisher.publish(msg)
            self.stop_task()

        elif command == "sleep":
            self.stop_task()
            msg = String()
            msg.data = "None"
            self.vision_publisher.publish(msg)

            if self.turtlebot4_navigator.getDockedStatus():
                self.get_logger().info("Already docked")
                return
            position = self.goals[command]["position"]
            goal_pose = self.turtlebot4_navigator.getPoseStamped(
                position, TurtleBot4Directions(self.goals[command]["orientation"])
            )
            self.get_logger().info("Navigating to docking station...")

            # Start new navigation task
            self.navigation_task = self.navigator.goToPose(goal_pose)

            self.save_state("go_to_goal")
            while not self.navigator.isTaskComplete():
                with open(self.state_path, "r") as file:
                    self.state = json.load(file)
                if self.state == "None":
                    self.get_logger().info("nav to goal interrupted...")
                    self.stop_task()
                    return

            self.turtlebot4_navigator.dock()

        elif command == "patrol":
            if self.turtlebot4_navigator.getDockedStatus():
                self.get_logger().info("currently docked, undocking...")
                self.turtlebot4_navigator.undock()

            self.get_logger().info("Starting patrol...")

            goal_pose = []
            for i in self.goals:
                msg = String()
                msg.data = "patrolling"
                self.vision_publisher.publish(msg)
                self.save_state("patrolling")

                position = self.goals[i]["position"]

                goal_pose = self.turtlebot4_navigator.getPoseStamped(
                    position, TurtleBot4Directions(self.goals[i]["orientation"])
                )
                # Start new navigation task
                self.navigation_task = self.navigator.goToPose(goal_pose)

                while not self.navigator.isTaskComplete():
                    with open(self.state_path, "r") as file:
                        self.state = json.load(file)
                    if self.state == "None":
                        self.get_logger().info("Patrol interrupted, stopping...")
                        self.stop_task()
                        return

        # Determine goal pose based on user input
        elif command == "wake up":
            if not self.turtlebot4_navigator.getDockedStatus():
                self.get_logger().info("Already undocked")
                return
            self.get_logger().info("Undocking...")
            self.turtlebot4_navigator.undock()
            return

        elif command in self.goals:
            msg = String()
            msg.data = "None"
            self.vision_publisher.publish(msg)

            self.get_logger().info("Navigating to " + command)
            if self.turtlebot4_navigator.getDockedStatus():
                self.get_logger().info("Undocking...")
                self.turtlebot4_navigator.undock()
            position = self.goals[command]["position"]
            goal_pose = self.turtlebot4_navigator.getPoseStamped(
                position, TurtleBot4Directions(self.goals[command]["orientation"])
            )

            # Stop current navigation task if any
            self.stop_task()

            # Start new navigation task
            self.navigation_task = self.navigator.goToPose(goal_pose)

        # elif command == "body detected":
        #     self.get_logger().info("Body detected, stopping current task...")
        #     self.stop_task()
        #     return

        # elif command == "lying body detected":
        #     self.get_logger().info("Lying body detected, stopping current task...")
        #     self.stop_task()
        #     return

        else:
            self.get_logger().warn(
                f"Invalid location entered({command}). Please enter a valid command from the following list: "
            )
            for i in self.goals:
                self.get_logger().info(i)
            return

    def save_state(self, state):
        with open(self.state_path, "w") as file:
            json.dump(state, file, indent=4)
            self.get_logger().info("state updated!")

    def stop_task(self):
        if self.navigation_task is not None:
            self.get_logger().info("Stopping current task...")
            self.navigator.cancelTask()
            self.navigation_task = None
        return


if __name__ == "__main__":
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
    rclpy.shutdown()
