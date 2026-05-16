import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import mavsdk


def generate_launch_description() -> LaunchDescription:
    mavlink_router_pkg = get_package_share_directory("mavlink_router")
    mavdrone_ros_pkg = get_package_share_directory("mavdrone_ros")

    mavsdk_server_file = os.path.join(os.path.dirname(mavsdk.__file__), "bin", "mavsdk_server")
    mavlink_router_launch_file = os.path.join(mavlink_router_pkg, "launch", "router.launch.py")
    mavdrone_params_file = os.path.join(mavdrone_ros_pkg, "config", "mavdrone_params.yaml")

    declare_mavdrone_params_file = DeclareLaunchArgument(
        "mavdrone_params_file",
        default_value=mavdrone_params_file,
        description="Path to the parameter file for the MavDrone node",
    )

    declare_camera_info_file = DeclareLaunchArgument(
        "camera_info_file",
        default_value="PACKAGE://mavdrone_ros/config/camera_info.yaml",
        description="Path to the camera info file",
    )
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="p1_camera",
        description="Camera name",
    )

    mavdrone_node = Node(
        package="mavdrone_ros",
        executable="mavdrone",
        name="mavdrone",
        output="screen",
        parameters=[
            LaunchConfiguration("mavdrone_params_file"),
            {"camera_info_file": LaunchConfiguration("camera_info_file"), "camera_name": LaunchConfiguration("camera_name")},
        ],
    )

    return LaunchDescription(
        [
            declare_mavdrone_params_file,
            declare_camera_info_file,
            declare_camera_name,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(mavlink_router_launch_file)),
            ExecuteProcess(cmd=[mavsdk_server_file, "-p", "5001"], output="screen"),
            mavdrone_node,
        ]
    )
