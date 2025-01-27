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
        "wall_boundary_segmentation", "param", "wall_boundary_segmentation.param.yaml"
    )
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            race_type_arg,
            Node(
                package="wall_boundary_segmentation",
                executable="wall_boundary_segmentation_node_exe",
                output="screen",
                parameters=[
                    param_file_path,
                    use_sim_time,
                ],
                remappings=[
                    ("right_wall_segment", "/perception/right_wall_segment"),
                    ("left_wall_segment", "/perception/left_wall_segment"),
                    ("right_wall_plane", "/perception/right_wall_plane"),
                    ("left_wall_plane", "/perception/left_wall_plane"),
                    ("right_wall_points", "/perception/right_wall_points"),
                    ("left_wall_points", "/perception/left_wall_points"),
                    ("right_downsampled_points", "/perception/right_downsampled_points"),
                    ("left_downsampled_points", "/perception/left_downsampled_points"),
                ],
            ),
        ]
    )
