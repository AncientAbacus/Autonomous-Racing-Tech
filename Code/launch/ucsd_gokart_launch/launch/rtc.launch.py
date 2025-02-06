from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("ucsd_gokart_launch")

    param_file_path = os.path.join(
        bringup_dir, "param", "rtc.param.yaml"
    )

    rtc_node = Node(
        package="gkc",
        executable="go_kart_controller",
        name="rtc_node",
        output="screen",
        parameters=[param_file_path],
        remappings=[('/gkc/cmd_vel', '/auto/raw_command')]
    )

    ld = LaunchDescription()
    ld.add_action(rtc_node)

    return ld

