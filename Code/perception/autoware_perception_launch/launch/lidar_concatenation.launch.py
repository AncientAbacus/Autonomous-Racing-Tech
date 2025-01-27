from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from base_common import get_share_file


def generate_launch_description():
    get_package_share_directory("autoware_perception_launch")

    lidar_topics = [
        "/luminar_right_points",
        "/luminar_left_points",
        "/luminar_front_points",
    ]

    # URDF robot state publisher
    vehicle_name_arg = DeclareLaunchArgument(
        name="urdf_path",
        default_value=get_share_file("race_metadata", "urdf", "iac_car", "av24.urdf"),
        description="Vehicle we are running",
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_path")]), value_type=str
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Input lidar topics
    input_points_raw_list_param = DeclareLaunchArgument(
        "input_points_raw_list",
        default_value="['/luminar_right_points', '/luminar_left_points', '/luminar_front_points']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )
    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")

    # Autoware pointcloud concatenation plugin
    concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_filter",
        remappings=[("output", "lidar_all")],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_points_raw_list"),
                "output_frame": LaunchConfiguration("tf_output_frame"),
                "approximate_sync": True,
                "timeout_sec": 0.3,
            }
        ],
    )
    pointcloud_concat_container = ComposableNodeContainer(
        name="pointcloud_concatenation_container",
        namespace="lidar_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[concat_component],
        output="screen",
    )

    return LaunchDescription(
        [
            input_points_raw_list_param,
            tf_output_frame_param,
            vehicle_name_arg,
            robot_state_publisher_node,
            pointcloud_concat_container,
        ]
    )
