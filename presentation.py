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


def main_menu():
    print("\nMain Menu:")
    print("1. Presentation Setup")
    print("2. Execute Command 2")
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
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
