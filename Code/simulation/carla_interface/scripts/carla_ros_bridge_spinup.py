#!/usr/bin/env python3

# """
# ROS2 Carla ROS Bridge Launcher
# https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/

# Note: Please ensure that the Carla ROS bridge package is built correctly before
# running this script.
# """

import subprocess
import argparse
import os

DEFAULT_PATH = os.path.expanduser("~")
# Linux: "/home/USER" Windows: "C:\Users\USERNAME"


def launch_carla_ros_bridge(path):
    # Path to the ROS2 workspace setup script
    setup_script_path = f"{path}/carla-ros-bridge/install/setup.bash"

    # Command to launch the Carla ROS bridge with the example ego vehicle
    launch_command = (
        "ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py"
    )

    # Command to source the setup script and then run the launch command
    command = f"source {setup_script_path} && {launch_command}"

    # Run the command
    subprocess.run(command, shell=True, executable="/bin/bash")


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="ROS2 Carla ROS Bridge Launcher")
    parser.add_argument(
        "--path",
        type=str,
        default=DEFAULT_PATH,
        help="Path to carla-ros-bridge package installation",
    )
    args = parser.parse_args()

    # Launch CARLA ROS bridge
    launch_carla_ros_bridge(args.path)


if __name__ == "__main__":
    main()
