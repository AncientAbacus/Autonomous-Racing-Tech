from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from base_common import get_share_file
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory("ucsd_gokart_launch")

    param_file_path = os.path.join(
        bringup_dir, "param", "livox.param.yaml"
    )

    config_filepath = get_share_file(
    "ucsd_gokart_launch", "param", "HAP_config.json"
    )

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_node",
        output="screen",
        parameters=[param_file_path, {'user_config_path': config_filepath}],
    )

    ld = LaunchDescription()
    ld.add_action(livox_node)

    return ld

