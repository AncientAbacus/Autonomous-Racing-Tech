#!/usr/bin/env python3

import subprocess
import argparse

DEFAULT_PATH = "/opt/carla-simulator"
DEFAULT_OS = "Linux"

# EXAMPLE RUN ARGUMENTS:
# python3 run_carla.py --path "C:\carla-simulator" --os Windows


def run_carla(path, operating_system):
    """
    Run CARLA Simulation

    This script launches the CARLA simulator using the specified path and operating system.
    If no path is provided, the default path '/opt/carla-simulator' is used.
    For Linux, the command is: cd {path} && ./CarlaUE4.sh -preferNvidia
    For Windows, the command is: cd {path} && CarlaUE4.exe
    """

    if operating_system.lower() == "windows":
        # Replace the command for Windows
        command = f"cd {path} && CarlaUE4.exe -preferNvidia -RenderOffScreen"
    else:
        # Command for Linux
        command = f"cd {path} && ./CarlaUE4.sh -preferNvidia -RenderOffScreen"

    # Run the command
    result = subprocess.run(command, shell=True, capture_output=True)

    # Check the return code of the subprocess command
    if result.returncode != 0:
        # Display the full error message
        print(f"Error running command: {command}")
        print(result.stderr.decode())

        # Check for specific launch errors
        if b"segfault" in result.stderr:
            print(
                "If you are experiencing a segfault, please ensure Vulkan or OpenGL drivers are installed."
            )
        else:
            print(
                "If you encounter any other issues, please refer to the CARLA Quickstart documentation:"
            )
            print("https://carla.readthedocs.io/en/latest/start_quickstart/")


if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run CARLA Simulation")
    parser.add_argument("--path", type=str, default=DEFAULT_PATH, help="Path to CARLA directory")
    parser.add_argument(
        "--os",
        type=str,
        choices=["Windows", "Linux"],
        default=DEFAULT_OS,
        help="Operating system (Windows, Linux)",
    )
    args = parser.parse_args()

    # Run CARLA with the specified path and operating system
    run_carla(args.path, args.os)
