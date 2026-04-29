"""Launch yolo_general node with config."""
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("yolo_general"), "config", "yolo_general.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="yolo_general",
                executable="yolo_general_node",
                name="yolo_general_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
