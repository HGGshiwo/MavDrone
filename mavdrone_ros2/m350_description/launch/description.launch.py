import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    m350_description_package = get_package_share_directory("m350_description")

    model_path = os.path.join(m350_description_package, "urdf", "m350.urdf.xacro")

    declare_model_path = DeclareLaunchArgument(name="model", default_value=model_path, description="Path to robot urdf file")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", LaunchConfiguration("model")]),
                "frame_prefix": "mavdrone/",
            }
        ],
    )

    return LaunchDescription(
        [
            declare_model_path,
            robot_state_publisher_node,
        ]
    )
