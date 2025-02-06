from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("ucsd_gokart_launch")

    param_file_path = os.path.join(
        bringup_dir, "param", "atlas.param.yaml"
    )

    atlas_node = Node(
        package="point_one_gps_driver",
        executable="point_one_gps_driver_node_exe",
        name="atlas_node",
        output="screen",
        parameters=[param_file_path],
        remappings={
            "nav_sat_fix": "/atlas/fix",
            "gps_fix": "/atlas/gps",
            "imu": "/atlas/imu",
            "pose": "/atlas/pose",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(atlas_node)

    return ld

