# Copyright 2023 Haoru Xue
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
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from environs import Env

env = Env()
env.read_env("race.env")


def get_share_file(package_name, *args):
    return os.path.join(get_package_share_directory(package_name), *args)


def get_sim_time_launch_arg():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )

    return declare_use_sim_time_cmd, {"use_sim_time": use_sim_time}


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    interface_config = get_share_file("art_mpc_interface", "param", "art_mpc_interface.param.yaml")
    mpc_config = get_share_file(
        "racing_lmpc_launch", "param", "racing_mpc", "iac_car_tracking_mpc.param.yaml"
    )
    dt_model_config = get_share_file(
        "racing_lmpc_launch", "param", "iac_car", "iac_car_single_track.param.yaml"
    )
    base_model_config = get_share_file(
        "racing_lmpc_launch", "param", "iac_car", "iac_car_base.param.yaml"
    )
    track_file_folder = get_share_file("racing_trajectory", "test_data", "LVMS")
    mpc_vd_model_name = DeclareLaunchArgument(
        "mpc_vehicle_model_name",
        default_value="single_track_planar_model",
        description="vehicle model name",
    )

    mpc_node = None
    if env.str("VEHICLE_NAME") == "IAC_CAR":
        mpc_node = Node(
            package="racing_mpc",
            executable="racing_mpc_node_exe",
            name="racing_mpc_node",
            output="screen",
            parameters=[
                mpc_config,
                dt_model_config,
                base_model_config,
                use_sim_time,
                {
                    "racing_mpc_node.vehicle_model_name": LaunchConfiguration(
                        "mpc_vehicle_model_name"
                    ),
                    "racing_mpc_node.default_traj_idx": 18,
                    "racing_mpc_node.traj_folder": track_file_folder,
                    "racing_mpc_node.velocity_profile_scale": 1.0,
                    "racing_mpc_node.delay_step": 1,
                },
            ],
            remappings=[],
            prefix=["nice -n -15"],
            emulate_tty=True,
        )
        mpc_solver_node = Node(
            package="racing_mpc",
            executable="racing_mpc_solver_node_exe",
            name="racing_mpc_solver_node",
            output="screen",
            parameters=[
                mpc_config,
                dt_model_config,
                base_model_config,
                use_sim_time,
                {
                    "racing_mpc_node.vehicle_model_name": LaunchConfiguration(
                        "mpc_vehicle_model_name"
                    ),
                },
            ],
            remappings=[
                ("solve_mpc", "mpc_0/solve_mpc"),
            ],
            prefix=["nice -n -15"],
            emulate_tty=True,
        )
    else:
        mpc_node = Node(
            package="racing_mpc",
            executable="racing_mpc_node_exe",
            name="racing_mpc_node",
            output="screen",
            parameters=[
                mpc_config,
                dt_model_config,
                base_model_config,
                use_sim_time,
                {
                    "racing_mpc_node.vehicle_model_name": LaunchConfiguration(
                        "mpc_vehicle_model_name"
                    ),
                    "racing_mpc_node.default_traj_idx": 18,
                    "racing_mpc_node.traj_folder": track_file_folder,
                    "racing_mpc_node.velocity_profile_scale": 1.0,
                    "racing_mpc_node.delay_step": 3,
                },
            ],
            remappings=[],
            emulate_tty=True,
        )
        mpc_solver_node = Node(
            package="racing_mpc",
            executable="racing_mpc_solver_node_exe",
            name="racing_mpc_solver_node",
            output="screen",
            parameters=[
                mpc_config,
                dt_model_config,
                base_model_config,
                use_sim_time,
                {
                    "racing_mpc_node.vehicle_model_name": LaunchConfiguration(
                        "mpc_vehicle_model_name"
                    ),
                },
            ],
            remappings=[
                ("solve_mpc", "mpc_0/solve_mpc"),
            ],
            emulate_tty=True,
        )

    interface_node = Node(
        package="art_mpc_interface",
        executable="art_mpc_interface_node_exe",
        name="art_mpc_interface_node",
        output="screen",
        parameters=[
            interface_config,
            dt_model_config,
            base_model_config,
            use_sim_time,
        ],
        remappings=[
            ("engine_report", "/vehicle/engine_report"),
            ("vehicle_kinematic_state", "/vehicle/state"),
            ("raw_cmd", "mpc_cmd"),
            ("target_trajectory_command", "/rde/trajectory_command"),
            ("rpp_path_command", "/rpp/trajectory_command"),
            ("steering_report", "/vehicle/steering_report"),
        ],
        prefix=["nice -15"],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            mpc_vd_model_name,
            interface_node,
            mpc_node,
            mpc_solver_node,
        ]
    )
