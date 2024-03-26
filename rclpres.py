import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


class RobotAssistant(Node):
    def __init__(self):
        super().__init__("robot_assistant")
        self.publisher_ = self.create_publisher(String, "/llm_plan", 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    robot_assistant = RobotAssistant()

    def execute_command(command):
        subprocess.Popen(command, shell=True)

    def initial():
        robot_assistant.get_logger().info(
            "Executing initial setup for the presentation..."
        )
        command = '{"plan":["use the navigation tool to wake up, dont use the speech tool for this","use the navigation tool to go to the stage","use the speech tool to say something like: hello everyone, my name is adam and I am delighted to welcome you to my presentation. I hope you enjoy the evening and have a great time. lets celebrate with a dance that ive been practicing","use the dance tool to perform a dance", "Use the speech tool to thank everyone and say that youre excited for the bytebot presentation", "Use the navigation tool to go to the stage"]}'
        robot_assistant.send_command(command)

    def patrol():
        robot_assistant.get_logger().info("Patrolling..")
        command = '{"plan":["say lets look at my patrolling abilities now, i will protect this store at all cost","send patrol to NavigationTool"]}'
        robot_assistant.send_command(command)

    def detect():
        robot_assistant.get_logger().info("Detected a person")
        command = '{"plan":["exclaim that a person has been detected","ask the person for a password - use the HumanInputTool, whatever they say, say that the password is wrong and initiate self destruction","use the speech tool to ask the person if they need help - you just noticed that they fell down and probably fainted, if theres no input from the HumanInputTool - exclaim that you tried everything you could","go to the stage"]}'
        robot_assistant.send_command(command)

    try:
        while True:
            print("\nMain Menu:")
            print("1. Presentation Setup")
            print("2. Patrol")
            print("3. Detect")
            print("0. Exit")
            choice = input("Enter your choice: ")

            if choice == "1":
                initial()
            elif choice == "2":
                patrol()
            elif choice == "3":
                detect()
            elif choice == "0":
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")
    except KeyboardInterrupt:
        pass

    robot_assistant.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
