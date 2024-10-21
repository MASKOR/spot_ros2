# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os

import launch
import launch_ros
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from spot_driver.launch.spot_launch_helpers import IMAGE_PUBLISHER_ARGS, declare_image_publisher_args, spot_has_arm

THIS_PACKAGE = "spot_driver"


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    spot_name = LaunchConfiguration("spot_name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    mock_enable = IfCondition(LaunchConfiguration("mock_enable", default="False")).evaluate(context)

    # if config_file has been set (and is not the default empty string) and is also not a file, do not launch anything.
    config_file_path = config_file.perform(context)
    if (config_file_path != "") and (not os.path.isfile(config_file_path)):
        raise FileNotFoundError("Configuration file '{}' does not exist!".format(config_file_path))

    if mock_enable:
        mock_has_arm = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)
        has_arm = mock_has_arm
    else:
        has_arm = spot_has_arm(config_file_path=config_file.perform(context), spot_name=spot_name)

    pkg_share = FindPackageShare("spot_description").find("spot_description")

    # Since spot_image_publisher_node is responsible for retrieving and publishing images, disable all image publishing
    # in spot_driver.
    spot_driver_params = {
        "spot_name": spot_name,
        "mock_enable": mock_enable,
    }

    if mock_enable:
        mock_spot_driver_params = {"mock_has_arm": mock_has_arm}
        # Merge the two dicts
        spot_driver_params = {**spot_driver_params, **mock_spot_driver_params}

    spot_driver_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_ros2",
        name="spot_ros2",
        output="screen",
        parameters=[config_file, spot_driver_params],
        namespace=spot_name,
    )
    ld.add_action(spot_driver_node)

    if not tf_prefix and spot_name:
        tf_prefix = PathJoinSubstitution([spot_name, ""])

    kinematc_node_params = {"spot_name": spot_name}
    kinematic_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_inverse_kinematics_node",
        output="screen",
        parameters=[config_file, kinematc_node_params],
        namespace=spot_name,
    )
    ld.add_action(kinematic_node)

    object_sync_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="object_synchronizer_node",
        output="screen",
        parameters=[config_file, {"spot_name": spot_name}],
        namespace=spot_name,
    )
    ld.add_action(object_sync_node)

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "spot.urdf.xacro"]),
            " ",
            "arm:=",
            TextSubstitution(text=str(has_arm).lower()),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )
    robot_description_params = {"robot_description": robot_description}
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_params],
        namespace=spot_name,
    )
    ld.add_action(robot_state_publisher)

    spot_robot_state_publisher_params = {"spot_name": spot_name, "preferred_odom_frame": "odom"}
    spot_robot_state_publisher = launch_ros.actions.Node(
        package="spot_driver",
        executable="state_publisher_node",
        output="screen",
        parameters=[config_file, spot_robot_state_publisher_params],
        namespace=spot_name,
    )
    ld.add_action(spot_robot_state_publisher)

    spot_alert_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_alerts",
        name="spot_alerts",
        output="screen",
        namespace=spot_name,
    )
    ld.add_action(spot_alert_node)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(THIS_PACKAGE), "/launch", "/rviz.launch.py"]),
        launch_arguments={
            "spot_name": spot_name,
            "rviz_config_file": rviz_config_file,
        }.items(),
        condition=IfCondition(launch_rviz),
    )
    ld.add_action(rviz)

    spot_image_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(THIS_PACKAGE), "/launch", "/spot_image_publishers.launch.py"]),
        launch_arguments={
            key: LaunchConfiguration(key) for key in ["config_file", "spot_name"] + IMAGE_PUBLISHER_ARGS
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_image_publishers")),
    )
    ld.add_action(spot_image_publishers)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to configuration file for the driver.",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="apply namespace prefix to robot links and joints",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="False",
            choices=["True", "true", "False", "false"],
            description="Choose whether to launch RViz",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description="RViz config file",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "launch_image_publishers",
            default_value="True",
            choices=["True", "true", "False", "false"],
            description="Choose whether to launch the image publishing nodes from Spot.",
        )
    )
    launch_args += declare_image_publisher_args()
    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
