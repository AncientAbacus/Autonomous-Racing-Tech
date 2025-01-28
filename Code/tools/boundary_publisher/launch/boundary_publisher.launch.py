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
from launch.actions import DeclareLaunchArgument
from base_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )
    config = get_share_file("boundary_publisher", "param", "boundary_publisher.param.yaml")
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            ttl_dir_arg,
            race_type_arg,
            Node(
                package="boundary_publisher",
                executable="boundary_publisher_node_exe",
                name="boundary_publisher_node",
                output="screen",
                parameters=[
                    config,
                    use_sim_time,
                    {"ttl_dir": LaunchConfiguration("ttl_dir")},
                ],
                namespace="bmp",
                remappings={"target_trajectory_command": "/rde/trajectory_command"}.items(),
                emulate_tty=True,
            ),
        ]
    )
