from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="carla_interface",
                executable="carla_interface_node",
                name="carla_interface_node",
                output="screen",
                remappings={"race_control": "/auto/raw_command"}.items(),
            ),
        ]
    )
