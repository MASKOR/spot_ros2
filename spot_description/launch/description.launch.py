# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> launch.LaunchDescription:
    pkg_share = FindPackageShare(package="spot_description").find("spot_description")
    default_model_path = os.path.join(pkg_share, "urdf/spot.urdf.xacro")
    default_rviz2_path = os.path.join(pkg_share, "rviz/viz_spot.rviz")
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
            ),
            DeclareLaunchArgument(
                name="rvizconfig", default_value=default_rviz2_path, description="Absolute path to rviz config file"
            ),
            DeclareLaunchArgument(
                name="arm",
                default_value="False",
                choices=["True", "true", "False", "false"],
                description="Flag to enable arm",
            ),
            DeclareLaunchArgument(
                name="feet",
                default_value="False",
                choices=["True", "true", "False", "false"],
                description="Flag to enable putting frames at the feet",
            ),
            DeclareLaunchArgument(
                "tf_prefix", default_value='""', description="Apply namespace prefix to robot links and joints"
            ),
            DeclareLaunchArgument("namespace", default_value="", description="Namespace for robot tf topic"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                LaunchConfiguration("model"),
                                " arm:=",
                                LaunchConfiguration("arm"),
                                " feet:=",
                                LaunchConfiguration("feet"),
                                " tf_prefix:=",
                                LaunchConfiguration("tf_prefix"),
                            ]
                        )
                    }
                ],
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
                namespace=LaunchConfiguration("namespace"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d" + default_rviz2_path],
            ),
        ]
    )
