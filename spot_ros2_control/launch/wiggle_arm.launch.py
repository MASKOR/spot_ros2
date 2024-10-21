# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spot_name",
                default_value="",
                description="Name of the Spot that will be used as a namespace.",
            ),
            Node(
                package="spot_ros2_control",
                executable="wiggle_arm",
                name="wiggle_arm",
                output="screen",
                parameters=[{"spot_name": LaunchConfiguration("spot_name")}],
                namespace=LaunchConfiguration("spot_name"),
            ),
        ]
    )
