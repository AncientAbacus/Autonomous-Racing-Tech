# Copyright 2021 AI Racing Tech

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from base_common import get_param_file, get_sim_time_launch_arg, check_val_in_list


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    vehicle_name_arg = DeclareLaunchArgument("vehicle_name")
    config = get_param_file("race_telemetry", "telemetry", "vehicle_name")

    tlm_node = Node(
        package="race_telemetry",
        executable="race_telemetry_node_exe",
        output="screen",
        parameters=[
            config,
            use_sim_time,
            {"ttl_directory": LaunchConfiguration("ttl_dir")},
        ],
        remappings=[
            ("telemetry", "/art_telemetry/telemetry"),
            ("race_misc_report", "/vehicle/misc_report"),
            ("race_fault_report", "/vehicle/fault_report"),
            ("control_command", "/auto/raw_command"),
            ("race_vehicle_command", "/rc_to_ct"),
            ("race_vehicle_status", "/vehicle/status"),
            ("race_fl_tire_pressure", "/vehicle/fl_tire_pressure"),
            ("race_fr_tire_pressure", "/vehicle/fr_tire_pressure"),
            ("race_rl_tire_pressure", "/vehicle/rl_tire_pressure"),
            ("race_rr_tire_pressure", "/vehicle/rr_tire_pressure"),
            ("race_wheel_speed_report", "/vehicle/wheel_speed_report"),
            ("race_wheel_potentiometer_report", "/vehicle/wheel_potentiometer_report"),
            ("race_wheel_strain_gauge_report", "/vehicle/wheel_strain_gauge_report"),
            ("race_engine_report", "/vehicle/engine_report"),
            ("race_ride_height_report", "/vehicle/ride_height_report"),
            ("brake_report", "/brake_report"),
            ("race_engine_pressures_report", "/vehicle/engine_pressures_report"),
            ("gps_fix", "/gps_bot/gps"),
            ("imu", "/gps_bot/imu"),
            ("vehicle_kinematic_state", "/vehicle/state"),
            ("steering_report", "/vehicle/steering_report"),
            ("slip_angle_report", "/vehicle/slip_angle_report"),
            ("push2pass_report", "/vehicle/push2pass_report"),
        ],
        emulate_tty=True,
        condition=IfCondition(
            check_val_in_list(
                "vehicle_name",
                ["IAC_CAR", "HAWAII_GOKART", "SVL_IAC_CAR", "AWSIM_IAC_CAR"],
            )
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(vehicle_name_arg)
    ld.add_action(tlm_node)
    return ld
