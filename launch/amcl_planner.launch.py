import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("my_planner_pkg")
    default_map = os.path.join(pkg_share, "maps", "flur.yaml")
    amcl_config = os.path.join(pkg_share, "config", "amcl_localization.yaml")

    map_file = LaunchConfiguration("map")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    bootstrap_map_odom = LaunchConfiguration("bootstrap_map_odom")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Absolute path to the map YAML file.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument(
                "bootstrap_map_odom",
                default_value="true",
                description="Publish a temporary identity map->odom TF until /initialpose is received.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": map_file,
                        "topic_name": "map",
                        "frame_id": map_frame,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
            Node(
                package="my_planner_pkg",
                executable="bootstrap_map_odom",
                name="bootstrap_map_odom",
                output="screen",
                condition=IfCondition(bootstrap_map_odom),
                parameters=[
                    {
                        "map_frame": map_frame,
                        "odom_frame": odom_frame,
                        "initialpose_topic": "/initialpose",
                        "shutdown_delay_s": 2.0,
                    }
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    amcl_config,
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame_id": map_frame,
                        "odom_frame_id": odom_frame,
                        "base_frame_id": base_frame,
                        "scan_topic": scan_topic,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["amcl"]},
                ],
            ),
            Node(
                package="my_planner_pkg",
                executable="planner_pf_node",
                name="planner_pf_node",
                output="screen",
                parameters=[
                    {
                        "map_yaml": "",
                        "map_topic": "/map",
                        "publish_map": False,
                        "map_frame": map_frame,
                        "base_frame": base_frame,
                        "goal_topic": goal_topic,
                        "scan_topic": scan_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                    }
                ],
            ),
        ]
    )
