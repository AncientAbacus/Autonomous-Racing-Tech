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
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from base_common import (
    get_param_file,
    get_sim_time_launch_arg,
    check_val_in_list,
    get_share_file,
    to_lower,
)


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir", default_value="None", description="TTL Directory to use"
    )
    controller_arg = DeclareLaunchArgument(
        "controller_type", default_value="None", description="Controller Type"
    )
    race_type_arg = DeclareLaunchArgument(
        "race_type", default_value="None", description="Race Type"
    )
    vehicle_model_config = (
        get_share_file("vehicle_model"),
        "/param/",
        to_lower(LaunchConfiguration("vehicle_name")),
        ".param.yaml",
    )
    config = get_param_file("ghost_car")
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            ttl_dir_arg,
            race_type_arg,
            controller_arg,
            Node(
                package="ghost_car",
                executable="ghost_car_node_exe",
                name="ghost_car_node",
                output="screen",
                parameters=[
                    config,
                    use_sim_time,
                    vehicle_model_config,
                    {"ttl_dir": LaunchConfiguration("ttl_dir")},
                    {"controller_type": LaunchConfiguration("controller_type")},
                ],
                remappings=[("tracked_objects", "/tracked_objects")],
                emulate_tty=True,
                condition=IfCondition(
                    check_val_in_list(
                        "race_type",
                        [
                            "IAC_LVMS",
                            "IAC_KS",
                            "SVL_LVMS",
                            "AW_SIM_IAC_LVMS",
                            "HAWAII_GOKART_AAIS",
                            "AW_SIM_HAWAII_GOKART_AAIS",
                        ],
                    )
                ),
            ),
        ]
    )
