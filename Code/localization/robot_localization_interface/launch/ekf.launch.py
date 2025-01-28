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
    ekf_param_file = (
        get_share_file("robot_localization_interface", "param"),
        "/",
        to_lower(LaunchConfiguration("race_type")),
        "/",
        LaunchConfiguration("ekf_param_file"),
    )
    ekf_interface_param_file = (
        get_share_file("robot_localization_interface", "param"),
        "/",
        to_lower(LaunchConfiguration("race_type")),
        "/",
        LaunchConfiguration("ekf_interface_param_file"),
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name=LaunchConfiguration("ekf_node_name"),
        output="screen",
        parameters=[ekf_param_file, use_sim_time],
        remappings=[
            ("/odometry/filtered", LaunchConfiguration("output_odom_topic")),
            ("/accel/filtered", LaunchConfiguration("output_accel_topic")),
        ],
        emulate_tty=True,
    )
    ekf_interface_node = Node(
        package="robot_localization_interface",
        executable="robot_localization_interface_node_exe",
        name=(LaunchConfiguration("ekf_node_name"), "_interface"),
        output="screen",
        parameters=[
            ekf_interface_param_file,
            use_sim_time,
            {"publish_tf": LaunchConfiguration("publish_tf")},
        ],
        remappings=[
            ("odom", LaunchConfiguration("output_odom_topic")),
            ("accel", LaunchConfiguration("output_accel_topic")),
            ("slip", LaunchConfiguration("output_slip_topic")),
            ("state", "/vehicle/state"),
            ("steering_report", "/vehicle/steering_report"),
        ],
        emulate_tty=True,
    )

    return LaunchDescription([declare_use_sim_time_cmd, ekf_node, ekf_interface_node])
