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
from launch.substitutions import LaunchConfiguration
from base_common import get_share_file, get_sim_time_launch_arg, to_lower


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    vehicle_model_config = (
        get_share_file("vehicle_model"),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        ".param.yaml",
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="velocity_estimation",
                executable="velocity_estimation_node_exe",
                name="velocity_estimation_node",
                output="screen",
                parameters=[
                    vehicle_model_config,
                    use_sim_time,
                ],
                remappings=[
                    ("wheel_speed_report", "/vehicle/wheel_speed_report"),
                    ("steering_report", "/vehicle/steering_report"),
                    ("twist", "/vehicle/twist"),
                ],
                emulate_tty=True,
            ),
        ]
    )
