# Copyright 2022 AI Racing Tech
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )
    launch_descriptions = [declare_use_sim_time_cmd]

    camera_topics = [
        (
            "/vehicle_8/camera/front_left/image/compressed",
            "/vehicle_8/camera/front_left/camera_info",
            "/vimba_front_left",
        ),
        (
            "/vehicle_8/camera/front_right/image/compressed",
            "/vehicle_8/camera/front_right/camera_info",
            "/vimba_front_right",
        ),
        (
            "/vehicle_8/camera/front_left_center/image/compressed",
            "/vehicle_8/camera/front_left_center/camera_info",
            "/vimba_front_left_center",
        ),
        # ("/vehicle_8/camera/front_right_center/image/compressed",
        #  "/vehicle_8/camera/front_right_center/camera_info", "/vimba_front_right_center"),
        (
            "/vehicle_8/camera/rear_left/image/compressed",
            "/vehicle_8/camera/rear_left/camera_info",
            "/vimba_rear_left",
        ),
        (
            "/vehicle_8/camera/rear_right/image/compressed",
            "/vehicle_8/camera/rear_right/camera_info",
            "/vimba_rear_right",
        ),
    ]

    for raw_topic, info_topic, converted_namespace in camera_topics:
        print(f"Adding camera topic {raw_topic}")
        name_suffix = converted_namespace.replace("/", "_")

        uncompress_node = Node(
            package="image_transport",
            executable="republish",
            output="screen",
            name="republish" + name_suffix,
            arguments=["compressed", "raw"],
            remappings=[("in/compressed", raw_topic), ("out", converted_namespace + "/image")],
        )

        composable_nodes = [
            ComposableNode(
                package="image_proc",
                plugin="image_proc::DebayerNode",
                name="debayer_node" + name_suffix,
                remappings=[
                    ("image_raw", converted_namespace + "/image"),
                    ("image_color", converted_namespace + "/debayered"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_color_node" + name_suffix,
                # Remap subscribers and publishers
                remappings=[
                    ("image", converted_namespace + "/debayered"),
                    ("image_rect", converted_namespace + "/rect"),
                    ("camera_info", info_topic),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]

        image_processing_container = ComposableNodeContainer(
            name="image_proc_container" + name_suffix,
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=composable_nodes,
            output="screen",
        )

        launch_descriptions.append(uncompress_node)
        launch_descriptions.append(image_processing_container)

    return launch.LaunchDescription(launch_descriptions)
