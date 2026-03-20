from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    map_topic = LaunchConfiguration("map_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    status_topic = LaunchConfiguration("status_topic")
    planner_max_lin = LaunchConfiguration("planner_max_lin")
    planner_max_ang = LaunchConfiguration("planner_max_ang")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
            DeclareLaunchArgument("status_topic", default_value="/planner_pf/status"),
            DeclareLaunchArgument(
                "planner_max_lin",
                default_value="0.35",
                description="Lower linear speed for autonomous exploration.",
            ),
            DeclareLaunchArgument(
                "planner_max_ang",
                default_value="1.0",
                description="Angular speed limit for autonomous exploration.",
            ),
            Node(
                package="my_planner_pkg",
                executable="planner_pf_node",
                name="planner_pf_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_yaml": "",
                        "map_topic": map_topic,
                        "publish_map": False,
                        "map_frame": map_frame,
                        "base_frame": base_frame,
                        "goal_topic": goal_topic,
                        "status_topic": status_topic,
                        "scan_topic": scan_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "max_lin": planner_max_lin,
                        "max_ang": planner_max_ang,
                    }
                ],
            ),
            Node(
                package="my_planner_pkg",
                executable="frontier_pf_explorer",
                name="frontier_pf_explorer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": map_frame,
                        "base_frame": base_frame,
                        "map_topic": map_topic,
                        "goal_topic": goal_topic,
                        "planner_status_topic": status_topic,
                    }
                ],
            ),
        ]
    )
