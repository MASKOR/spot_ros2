# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spot_name",
                default_value="",
                description="Name of the Spot that will be used as a namespace.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("spot_ros2_control"), "launch", "spot_ros2_control.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "mock_has_arm": "true",
                    "robot_controller": "forward_position_controller",
                    "hardware_interface": "mock",
                    "spot_name": LaunchConfiguration("spot_name"),
                }.items(),
            ),
            Node(
                package="spot_ros2_control",
                executable="wiggle_arm",
                name="wiggle_arm",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("spot_ros2_control"), "config", "examples.yaml"])],
                namespace=LaunchConfiguration("spot_name"),
            ),
        ]
    )
