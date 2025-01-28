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
from launch.actions import DeclareLaunchArgument
from base_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    param_file_path = get_share_file(
        "race_boundary_localizer", "param", "race_boundary_localizer.param.yaml"
    )
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            race_type_arg,
            Node(
                package="race_boundary_localizer",
                executable="race_boundary_localizer_node_exe",
                output="screen",
                parameters=[
                    param_file_path,
                    use_sim_time,
                ],
                remappings=[
                    ("distance_to_wall", "/localization/right_wall_distance"),
                    ("left_wall_distance", "/localization/left_wall_distance"),
                    ("heading_from_wall", "/localization/heading"),
                ],
            ),
        ]
    )
