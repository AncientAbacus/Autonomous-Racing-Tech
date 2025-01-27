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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_file = DeclareLaunchArgument(
        "tracking_param_file",
        default_value=os.path.join(
            get_package_share_directory("autoware_perception_launch"), "param/tracker.param.yaml"
        ),
    )
    data_association_matrix_file_param = DeclareLaunchArgument(
        "data_association_matrix_file",
        default_value=os.path.join(
            get_package_share_directory("autoware_perception_launch"),
            "param/data_association_matrix.param.yaml",
        ),
    )
    distance_threshold_list_param = DeclareLaunchArgument(
        "distance_threshold_list_file",
        default_value=os.path.join(
            get_package_share_directory("autoware_perception_launch"),
            "param/distance_threshold_list.param.yaml",
        ),
    )

    # set container to run all required components in the same process
    tracker_node = Node(
        name="multi_object_tracker",
        package="multi_object_tracker",
        executable="multi_object_tracker",
        remappings=[
            ("input", "/merged_objects"),
            ("output", "tracked_objects"),
        ],
        parameters=[
            LaunchConfiguration("tracking_param_file"),
            LaunchConfiguration("data_association_matrix_file"),
        ],
        output="screen",
    )

    # detection feedback
    detection_by_tracker_node = Node(
        name="detection_by_tracker_node",
        package="detection_by_tracker",
        executable="detection_by_tracker",
        remappings=[
            ("/detection_by_tracker_node/input/tracked_objects", "/tracked_objects"),
            ("/detection_by_tracker_node/input/initial_objects", "/clusters_with_feature"),
            ("/detection_by_tracker_node/output", "/tracker_detected_objects"),
        ],
        parameters=[{"ignore_unknown_tracker": True}],
        output="screen",
    )

    # merge detection feedback with new results
    tracker_detection_merger_node = Node(
        name="tracker_detection_merger_node",
        package="object_merger",
        executable="object_association_merger_node",
        remappings=[
            ("input/object0", "/tracker_detected_objects"),
            ("input/object1", "/objects"),
            ("output/object", "/merged_objects"),
        ],
        parameters=[
            {
                "priority_mode": 0,  # 0 for object 0, 1 for object 1, 2 for confidence
                "generalized_iou_threshold": -0.6,
                "precision_threshold_to_judge_overlapped": 0.4,
            },
            LaunchConfiguration("data_association_matrix_file"),
            LaunchConfiguration("distance_threshold_list_file"),
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            param_file,
            data_association_matrix_file_param,
            distance_threshold_list_param,
            tracker_node,
            detection_by_tracker_node,
            tracker_detection_merger_node,
        ]
    )
