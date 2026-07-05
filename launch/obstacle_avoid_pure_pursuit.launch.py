from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("obstacle_avoid_pure_pursuit"),
        "config",
        "obstacle_avoid_pure_pursuit.yaml",
    ])

    return LaunchDescription([
        Node(
            package="obstacle_avoid_pure_pursuit",
            executable="obstacle_avoid_pure_pursuit_node",
            name="obstacle_avoid_pure_pursuit",
            output="screen",
            parameters=[params],
        )
    ])
