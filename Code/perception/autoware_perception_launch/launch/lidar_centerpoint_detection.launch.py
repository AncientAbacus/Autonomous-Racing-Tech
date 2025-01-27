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

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # declare launch arguments
    pkg = get_package_share_directory("autoware_perception_launch")
    input_points_raw_list_param = DeclareLaunchArgument(
        "input_points_raw_list",
        default_value="['/livox/lidar/front_left', '/livox/lidar/front_center', '/livox/lidar/front_right']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )

    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")
    param_file = DeclareLaunchArgument(
        "lidar_centerpoint_param_file",
        default_value=os.path.join(pkg, "param/lidar_centerpoint.param.yaml"),
    )
    output_topic_param = DeclareLaunchArgument("output_objects", default_value="objects")

    encoder_onnx_param = DeclareLaunchArgument(
        "encoder_onnx_path",
        default_value=os.path.join(pkg, "config", "model", "pts_voxel_encoder_centerpoint.onnx"),
    )
    encoder_engine_param = DeclareLaunchArgument(
        "encoder_engine_path",
        default_value=os.path.join(pkg, "config", "model", "pts_voxel_encoder_centerpoint.engine"),
    )
    head_onnx_param = DeclareLaunchArgument(
        "head_onnx_path",
        default_value=os.path.join(
            pkg, "config", "model", "pts_backbone_neck_head_centerpoint.onnx"
        ),
    )
    head_engine_param = DeclareLaunchArgument(
        "head_engine_path",
        default_value=os.path.join(
            pkg, "config", "model", "pts_backbone_neck_head_centerpoint.engine"
        ),
    )

    # set concat filter as a component
    concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_filter",
        remappings=[("output", "points_raw/concatenated")],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_points_raw_list"),
                "output_frame": LaunchConfiguration("tf_output_frame"),
                "approximate_sync": True,
            }
        ],
    )

    # set container to run all required components in the same process
    concat_container = ComposableNodeContainer(
        name="pointcloud_detection_container",
        namespace="lidar_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[concat_component],
        output="screen",
    )

    lidar_centerpoint_node = Node(
        name="lidar_centerpoint_node",
        package="lidar_centerpoint",
        executable="lidar_centerpoint_node",
        remappings=[
            ("~/input/pointcloud", "points_raw/concatenated"),
            ("~/output/objects", LaunchConfiguration("output_objects")),
        ],
        parameters=[
            LaunchConfiguration("lidar_centerpoint_param_file"),
            {
                "encoder_onnx_path": LaunchConfiguration("encoder_onnx_path"),
                "encoder_engine_path": LaunchConfiguration("encoder_engine_path"),
                "head_onnx_path": LaunchConfiguration("head_onnx_path"),
                "head_engine_path": LaunchConfiguration("head_engine_path"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    # check the size of input_points_raw_list
    log_info = LogInfo(
        msg=PythonExpression(
            [
                "'input_points_raw_list size = ' + str(len(",
                LaunchConfiguration("input_points_raw_list"),
                "))",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            input_points_raw_list_param,
            tf_output_frame_param,
            param_file,
            output_topic_param,
            encoder_onnx_param,
            encoder_engine_param,
            head_onnx_param,
            head_engine_param,
            lidar_centerpoint_node,
            concat_container,
            log_info,
        ]
    )
