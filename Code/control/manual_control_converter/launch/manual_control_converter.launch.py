# Copyright 2022 Siddharth Saha


from launch import LaunchDescription
from launch_ros.actions import Node
from base_common import get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="manual_control_converter",
                executable="manual_control_converter_node_exe",
                name="manual_control_converter_node",
                output="screen",
                parameters=[use_sim_time],
                remappings=[
                    ("manual_cmd", "joystick/control_command"),
                    ("cmd", "/auto/raw_command"),
                ],
                emulate_tty=True,
            ),
        ]
    )
