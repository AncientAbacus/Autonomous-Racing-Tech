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
    pkg = get_package_share_directory("autoware_perception_launch")
    param_file = DeclareLaunchArgument(
        "lidar_detection_param_file",
        default_value=os.path.join(pkg, "param", "yolov5l.param.yaml"),
    )
    input_topic_param = DeclareLaunchArgument(
        "input_topic", default_value="/vimba_front_left_center/rect"
    )
    output_det_topic_param = DeclareLaunchArgument(
        "output_detection", default_value="/vimba_front_left_center/detection"
    )
    output_img_topic_param = DeclareLaunchArgument(
        "output_image", default_value="/vimba_front_left_center/detection_vis"
    )
    onnx_file_param = DeclareLaunchArgument(
        "onnx_file", default_value=os.path.join(pkg, "config", "model", "yolov5m.onnx")
    )
    engine_file_param = DeclareLaunchArgument(
        "engine_file", default_value=os.path.join(pkg, "config", "model", "yolov5m.engine")
    )
    label_file_param = DeclareLaunchArgument(
        "label_file", default_value=os.path.join(pkg, "config", "model", "coco.names")
    )

    yolo_node = Node(
        package="tensorrt_yolo",
        executable="tensorrt_yolo_node",
        name="tensorrt_yolo",
        remappings=[
            ("in/image", LaunchConfiguration("input_topic")),
            ("out/objects", LaunchConfiguration("output_detection")),
            ("out/image", LaunchConfiguration("output_image")),
        ],
        parameters=[
            LaunchConfiguration("lidar_detection_param_file"),
            {
                "onnx_file": LaunchConfiguration("onnx_file"),
                "engine_file": LaunchConfiguration("engine_file"),
                "label_file": LaunchConfiguration("label_file"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    return launch.LaunchDescription(
        [
            param_file,
            input_topic_param,
            output_det_topic_param,
            output_img_topic_param,
            onnx_file_param,
            engine_file_param,
            label_file_param,
            yolo_node,
        ]
    )
