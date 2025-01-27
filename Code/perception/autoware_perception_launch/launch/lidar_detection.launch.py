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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # declare launch arguments
    input_points_raw_list_param = DeclareLaunchArgument(
        "input_points_raw_list",
        default_value="['/livox/lidar/front_left', '/livox/lidar/front_center', '/livox/lidar/front_right']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )

    output_clusters_param = DeclareLaunchArgument("output_objects", default_value="objects")

    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")

    param_file = DeclareLaunchArgument(
        "lidar_detection_param_file",
        default_value=os.path.join(
            get_package_share_directory("autoware_perception_launch"),
            "param/lidar_preprocessor.param.yaml",
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

    # set euclidean clustering as a component
    euclidean_cluster_component = ComposableNode(
        package="euclidean_cluster",
        plugin="euclidean_cluster::EuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", "points_raw/concatenated"),
            ("output", "clusters_with_feature"),
        ],
        parameters=[LaunchConfiguration("lidar_detection_param_file")],
    )

    # set shape estimation node as a component
    shape_estimation_component = ComposableNode(
        package="shape_estimation",
        plugin="ShapeEstimationNode",
        name="shape_estimation",
        remappings=[
            ("input", "clusters_with_feature"),
            ("objects", "shape_estimation"),
        ],
        parameters=[LaunchConfiguration("lidar_detection_param_file")],
    )

    # set message converter node as a component
    feature_remover_component = ComposableNode(
        package="detected_object_feature_remover",
        plugin="detected_object_feature_remover::DetectedObjectFeatureRemover",
        name="detected_object_feature_remover",
        remappings=[
            ("~/input", "shape_estimation"),
            ("~/output", LaunchConfiguration("output_objects")),
        ],
        parameters=[LaunchConfiguration("lidar_detection_param_file")],
    )

    # set container to run all required components in the same process
    detection_container = ComposableNodeContainer(
        name="pointcloud_detection_container",
        namespace="lidar_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[concat_component, euclidean_cluster_component],
        output="screen",
    )

    estimation_container = ComposableNodeContainer(
        name="pointcloud_estimation_container",
        namespace="lidar_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[shape_estimation_component, feature_remover_component],
        output="screen",
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
            output_clusters_param,
            param_file,
            detection_container,
            estimation_container,
            log_info,
        ]
    )
