import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    m350_description_pkg = get_package_share_directory("m350_description")
    mavdrone_ros_pkg = get_package_share_directory("mavdrone_ros")

    mavdrone_launch_file = os.path.join(mavdrone_ros_pkg, "launch", "mavdrone.launch.py")
    local_grid_params_file = os.path.join(mavdrone_ros_pkg, "config", "local_grid_params.yaml")
    rviz_config_file = os.path.join(mavdrone_ros_pkg, "config", "default.rviz")
    description_launch_file = os.path.join(m350_description_pkg, "launch", "description.launch.py")

    declare_use_rviz = DeclareLaunchArgument("rviz", default_value="false", description="Whether to launch RViz")
    declare_use_drone = DeclareLaunchArgument("drone", default_value="false", description="Whether to launch RViz")
    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_config_file,
        description="Path to the RViz configuration file",
    )

    declare_local_grid_params_file = DeclareLaunchArgument(
        "local_grid_params_file",
        default_value=local_grid_params_file,
        description="Path to the parameter file for the local grid",
    )

    local_grid_node = Node(
        package="mavdrone_ros",
        executable="local_grid_broadcaster",
        name="local_grid_broadcaster",
        output="screen",
        parameters=[LaunchConfiguration("local_grid_params_file")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    goal_pose_server_node = Node(
        package="mavdrone_ros",
        executable="goal_pose_server_node",
        name="goal_pose_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_use_drone,
            declare_local_grid_params_file,
            declare_rviz_config_file,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mavdrone_launch_file), condition=IfCondition(LaunchConfiguration("drone"))
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_file)),
            local_grid_node,
            rviz_node,
            goal_pose_server_node,
        ]
    )