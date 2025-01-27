from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

# IF NEEDED TO MATCH YOUR SYSTEM, CHANGE THE FOLLOWING VARIABLES:
# TODO: edit to use param.yml file instead of hardcoding them here

# - CARLA_SIM_PATH: Specify the path to the CARLA Simulator directory.
# - OS: Specify the operating system (Windows or Linux).
# - CARLA_ROS_BRIDGE_PATH: Specify the path to the CARLA ROS bridge package directory.

# DEFAULT VALUES:

# CARLA_SIM_PATH Default: "/opt/carla-simulator"
# OS Default: "Linux"
# CARLA_ROS_BRIDGE_PATH Default:
#     - Linux: "/home/USER"
#     - Windows: "C:\Users\USERNAME"

CUSTOMIZE = True
CARLA_SIM_PATH = "/opt/carla-simulator"
OS = "Linux"
CARLA_ROS_BRIDGE_PATH = "/home/ckwolfe"


def generate_launch_description():
    package_share_directory = get_package_share_directory("carla_interface")

    run_carla_script = str(package_share_directory) + "/scripts/run_carla.py"
    carla_ros_bridge_spinup_script = (
        str(package_share_directory) + "/scripts/carla_ros_bridge_spinup.py"
    )

    launch_description = []

    # Run run_carla.py with customizable arguments if CUSTOMIZE is set to True
    if CUSTOMIZE:
        launch_description.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    run_carla_script,
                    "--path",
                    LaunchConfiguration("path", default=CARLA_SIM_PATH),
                    "--os",
                    LaunchConfiguration("os", default=OS),
                ],
                output="screen",
                additional_env={"PYTHONUNBUFFERED": "1"},
            )
        )
    else:
        # Run run_carla.py with no arguments
        launch_description.append(
            ExecuteProcess(
                cmd=["python3", run_carla_script],
                output="screen",
                additional_env={"PYTHONUNBUFFERED": "1"},
            )
        )

    # Run carla_ros_bridge_spinup.py with customizable arguments if CUSTOMIZE is set to True
    if CUSTOMIZE:
        launch_description.append(
            ExecuteProcess(
                cmd=["python3", carla_ros_bridge_spinup_script, "--path", CARLA_ROS_BRIDGE_PATH],
                output="screen",
                additional_env={"PYTHONUNBUFFERED": "1"},
            )
        )
    else:
        # Run carla_ros_bridge_spinup.py with no arguments
        launch_description.append(
            ExecuteProcess(
                cmd=["python3", carla_ros_bridge_spinup_script],
                output="screen",
                additional_env={"PYTHONUNBUFFERED": "1"},
            )
        )

    return LaunchDescription(launch_description)
