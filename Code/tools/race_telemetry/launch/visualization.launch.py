# Copyright 2024 AI Racing Tech

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from base_common import get_param_file, get_sim_time_launch_arg, check_val_in_list, get_share_file
from environs import Env
import os

env = Env()
env.read_env("race.env")


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")
    config = get_param_file("race_telemetry", "telemetry", "vehicle_name")
    vehicle_name_arg = DeclareLaunchArgument(
        "vehicle_name",
        default_value=env.str("VEHICLE_NAME"),
        description="Vehicle name",
    )
    print(os.path.join(config[0], env.str("TTL_FOLDER")))
    ttl_dir_arg = DeclareLaunchArgument(
        "ttl_dir",
        default_value=get_share_file("race_metadata", "ttls", env.str("TTL_FOLDER")),
        description="TTL directory",
    )

    visualization_node = Node(
        package="race_telemetry",
        executable="visualization_node_exe",
        output="screen",
        parameters=[
            config,
            use_sim_time,
            {"ttl_directory": LaunchConfiguration("ttl_dir")},
        ],
        remappings=[
            ("telemetry", "/art_telemetry/telemetry"),
        ],
        emulate_tty=True,
        condition=IfCondition(
            check_val_in_list(
                "vehicle_name",
                ["IAC_CAR", "HAWAII_GOKART", "SVL_IAC_CAR", "AWSIM_IAC_CAR"],
            )
        ),
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            vehicle_name_arg,
            vehicle_name_arg,
            ttl_dir_arg,
            visualization_node,
        ]
    )
