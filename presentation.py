import os
import subprocess


def handle_command_1():
    print("Executing Command 1...")


def handle_command_2():
    print("Executing Command 2...")

def initial():
    print("Executing initial setup for the presentation...")
    command = "ros2 topic pub /llm_plan std_msgs/msg/String \"data: '{\\\"plan\\\":[\\\"undock\\\",\\\"use the navigation tool to go to the stage\\\",\\\"use the speech tool to say something like: hello everyone, my name is adam and I am delighted to welcome you to my presentation. I hope you enjoy the evening and have a great time. lets celebrate with a dance that ive been practicing\\\", \\\"use the dance tool to perform a dance\\\"]}'\" -1"

    for _ in range(5):
        subprocess.run(command, shell=True)
        # subprocess.Popen(["bash", "-c", command])


def patrol():
    print("Executing initial setup for the presentation...")
    command = 'ros2 topic pub /llm_plan std_msgs/msg/String "data: \'{\\"plan\\":[\\"use the navigation tool to go to the stage\\",\\"use the speech tool to say something like: hello everyone, my name is adam and I am delighted to welcome you to my presentation. I hope you enjoy the evening and have a great time. lets celebrate with a dance that ive been practicing\\", \\"use the dance tool to perform a dance\\"]}\'" -1'

    for _ in range(5):
        subprocess.run(command, shell=True)


def detect():
    print("Detected a person")
    command = 'ros2 topic pub /llm_plan std_msgs/msg/String "data: \'{\\"plan\\":[\\"exclaim that a person has been detected\\",\\"ask the person for a password - use the HumanInputTool, whatever they say, say that the password is wrong and initiate self destruction\\",\\"use the speech tool to ask the person if they need help - you just noticed that they fell down and probably fainted, if theres no input from the HumanInputTool - exclaim that you tried everything you could\\", \\"go to the stage\\"]}\'" -1'

    for _ in range(5):
        subprocess.run(command, shell=True)


def main_menu():
    print("\nMain Menu:")
    print("1. Presentation Setup")
    print("2. Patrol")
    print("3. Detect")
    print("0. Exit")


def main():
    while True:
        main_menu()
        choice = input("Enter your choice: ")

        if choice == "1":
            initial()
        elif choice == "2":
            handle_command_2()
        elif choice == "0":
            print("Exiting...")
            break
        elif choice == "3":
            detect()
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
