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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from base_common import get_param_file


def generate_launch_description():
    vehicle_name_arg = DeclareLaunchArgument(
        "vehicle_name", default_value="None", description="Vehicle we are working with"
    )

    param_file = get_param_file(
        "art_autoware_interface", launch_config_var="vehicle_name", prefix=True
    )

    vehicle_info_param_file = get_param_file(
        "art_autoware_interface", "kart_vehicle_info", "vehicle_name", True
    )

    vehicle_characteristics_param_file = get_param_file(
        "art_autoware_interface", "kart_vehicle_characteristics", "vehicle_name", True
    )

    simulator_model_param_file = get_param_file(
        "art_autoware_interface", "kart_simple_planning_simulator", "vehicle_name", True
    )

    initial_engage_state_param = DeclareLaunchArgument(
        "initial_engage_state", default_value="true"
    )

    art_autoware_node = Node(
        name="art_autoware_interface_node",
        package="art_autoware_interface",
        executable="art_autoware_interface",
        remappings=[
            ("autoware/ackermann_control_command", "/control/command/control_cmd"),
            ("/odom", "/localization/kinematic_state"),
            ("autoware/steering_report", "/vehicle/status/steering_status"),
            ("art/steering_report", "/vehicle/steering_report"),
            ("art/control_command", "/auto/raw_command"),
        ],
        parameters=[
            param_file,
        ],
        output="screen",
    )

    simple_simulator_launch = GroupAction(
        actions=[
            SetRemap(src="input/initialpose", dst="/initialpose"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("simple_planning_simulator"),
                        "launch",
                        "simple_planning_simulator.launch.py",
                    )
                ),
                launch_arguments=(
                    ("vehicle_info_param_file", vehicle_info_param_file),
                    (
                        "vehicle_characteristics_param_file",
                        vehicle_characteristics_param_file,
                    ),
                    ("simulator_model_param_file", simulator_model_param_file),
                    ("initial_engage_state", LaunchConfiguration("initial_engage_state")),
                ),
            ),
        ]
    )

    return launch.LaunchDescription(
        [
            vehicle_name_arg,
            initial_engage_state_param,
            art_autoware_node,
            simple_simulator_launch,
        ]
    )
