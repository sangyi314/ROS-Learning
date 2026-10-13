from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = str(
        Path(get_package_share_directory("robot_cpp_basics"))
        / "config"
        / "sensors.yaml"
    )
    return LaunchDescription([
        Node(
            package="robot_cpp_basics",
            executable="sensor_publisher",
            name="sensor_publisher",
            namespace="robot1",
            parameters=[config_path],
            output="screen",
        ),
        Node(
            package="robot_cpp_basics",
            executable="sensor_processor",
            name="sensor_processor",
            namespace="robot1",
            parameters=[config_path],
            output="screen",
        ),
        Node(
            package="robot_cpp_basics",
            executable="collect_server",
            name="collect_server",
            namespace="robot1",
            output="screen",
        ),
    ])