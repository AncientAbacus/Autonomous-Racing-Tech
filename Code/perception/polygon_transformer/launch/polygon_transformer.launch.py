from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from base_common import get_share_file, get_sim_time_launch_arg, to_lower


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    config_param = (
        get_share_file("polygon_transformer"),
        "/param/polygon_remover_",
        to_lower(LaunchConfiguration("race_type")),
        ".param.yaml",
    )
    inside_poly = (
        get_share_file("polygon_transformer"),
        "/param/",
        to_lower(LaunchConfiguration("race_type")),
        "_inside.csv",
    )
    outside_poly = (
        get_share_file("polygon_transformer"),
        "/param/",
        to_lower(LaunchConfiguration("race_type")),
        "_outside.csv",
    )
    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="polygon_transformer",
                name="inside_boundary_transformer",
                executable="polygon_transformer_node_exe",
                parameters=[
                    config_param,
                    use_sim_time,
                    {
                        "polygon_csv": inside_poly,
                    },
                ],
                remappings=[
                    ("pc_remover_polygon_vis", "/perception/lvms_inside_vis"),
                    ("pc_remover_polygon", "/perception/lvms_inside"),
                    ("state", "/vehicle/state"),
                ],
            ),
            Node(
                package="polygon_transformer",
                name="outside_boundary_transformer",
                executable="polygon_transformer_node_exe",
                parameters=[
                    config_param,
                    use_sim_time,
                    {
                        "polygon_csv": outside_poly,
                    },
                ],
                remappings=[
                    ("pc_remover_polygon_vis", "/perception/lvms_outside_vis"),
                    ("pc_remover_polygon", "/perception/lvms_outside"),
                    ("state", "/vehicle/state"),
                ],
            ),
        ]
    )
