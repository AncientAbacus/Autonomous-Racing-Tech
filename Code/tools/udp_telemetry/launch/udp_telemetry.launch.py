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


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from base_common import get_param_file, get_sim_time_launch_arg, check_val_in_list


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")

    config = get_param_file("udp_telemetry", launch_config_var="vehicle_name")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            vehicle_name_arg,
            Node(
                package="udp_telemetry",
                executable="udp_telemetry_node_exe",
                name="udp_telemetry_node",
                output="screen",
                parameters=[config, use_sim_time],
                remappings=[
                    ("control_command", "/joystick/control_command"),
                    ("telemetry", "/art_telemetry/telemetry"),
                ],
                emulate_tty=True,
                condition=IfCondition(
                    check_val_in_list("vehicle_name", ["IAC_CAR", "HAWAII_GOKART"])
                ),
            ),
        ]
    )
