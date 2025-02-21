# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import os

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

THIS_PACKAGE = "spot_driver"


def create_rviz_config(robot_name: str, tf_prefix: str) -> None:
    """Writes a configuration file for rviz to visualize a single spot robot"""

    RVIZ_TEMPLATE_FILENAME = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "spot_template.yaml")
    RVIZ_OUTPUT_FILENAME = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "spot.rviz")

    with open(RVIZ_TEMPLATE_FILENAME, "r") as template_file:
        config = yaml.safe_load(template_file)

        if tf_prefix:
            # replace fixed frame with robot body frame
            config["Visualization Manager"]["Global Options"]["Fixed Frame"] = f"{tf_prefix}vision"
        if robot_name:
            # Add robot models for each robot
            for display in config["Visualization Manager"]["Displays"]:
                if "RobotModel" in display["Class"]:
                    display["Description Topic"]["Value"] = f"/{robot_name}/robot_description"

                if "Image" in display["Class"]:
                    topic_name = display["Topic"]["Value"]
                    display["Topic"]["Value"] = f"/{robot_name}{topic_name}"

    with open(RVIZ_OUTPUT_FILENAME, "w") as out_file:
        yaml.dump(config, out_file)


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    spot_name = LaunchConfiguration("spot_name").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    # It looks like passing an optional of value "None" gets converted to a string of value "None"
    if not rviz_config_file or rviz_config_file == "None":
        create_rviz_config(spot_name, tf_prefix)
        rviz_config_file = PathJoinSubstitution([FindPackageShare(THIS_PACKAGE), "rviz", "spot.rviz"]).perform(context)

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    ld.add_action(rviz)


def generate_launch_description() -> launch.LaunchDescription:
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="",
            description="RViz config file",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="apply namespace prefix to robot links and joints",
        )
    )
    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
