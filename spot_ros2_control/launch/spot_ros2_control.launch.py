# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
import os
from tempfile import NamedTemporaryFile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from spot_driver.launch.spot_launch_helpers import get_login_parameters, spot_has_arm

THIS_PACKAGE = "spot_ros2_control"


def create_controllers_config(spot_name: str, has_arm: bool) -> None:
    """Writes a configuration file used to put the ros2 control nodes into a namespace.
    This is necessary as if your ros2 control nodes are launched in a namespace, the configuration yaml used
    must also reflect this same namespace when defining parameters of your controllers.

    Args:
        spot_name (str): Name of spot. If it's the empty string, the default controller file with no namespace is used.
        has_arm (bool): Whether or not your robot has an arm. Necessary for defining the joints that the forward
                        position controller should use.

    Returns:
        str: Path to controllers config file to use
    """

    arm_text = "with_arm" if has_arm else "without_arm"
    template_filename = os.path.join(
        get_package_share_directory(THIS_PACKAGE), "config", f"spot_default_controllers_{arm_text}.yaml"
    )

    if spot_name:
        with open(template_filename, "r") as template_file:
            config = yaml.safe_load(template_file)
            forward_position_controller_joints = config["forward_position_controller"]["ros__parameters"]["joints"]
            config["forward_position_controller"]["ros__parameters"]["joints"] = [
                f"{spot_name}/{joint}" for joint in forward_position_controller_joints
            ]
            config[f"{spot_name}/controller_manager"] = config["controller_manager"]
            del config["controller_manager"]
            config[f"{spot_name}/forward_position_controller"] = config["forward_position_controller"]
            del config["forward_position_controller"]

        with NamedTemporaryFile(suffix=".yaml", mode="w", delete=False) as out_file:
            yaml.dump(config, out_file)
            return out_file.name
    else:
        # We do not need to do anything -- the template filename is the default for no namespace.
        return template_filename


def create_rviz_config(spot_name: str) -> str:
    """Writes a configuration file for rviz to visualize a robot launched in a namespace. This is necessary as you need
    to specify different topics in the rviz config file depending on the namespace.

    Args:
        spot_name (str): Name of Spot corresponding to the namespace the nodes are launched in.

    Returns:
        str: Path to RViz config file to use
    """

    template_filename = os.path.join(get_package_share_directory(THIS_PACKAGE), "rviz", "template.rviz")

    if spot_name:
        with open(template_filename, "r") as template_file:
            config = yaml.safe_load(template_file)
            # replace fixed frame with robot body frame
            config["Visualization Manager"]["Global Options"]["Fixed Frame"] = f"{spot_name}/body"
            # Add robot models for each robot
            for display in config["Visualization Manager"]["Displays"]:
                if "RobotModel" in display["Class"]:
                    display["Description Topic"]["Value"] = f"/{spot_name}/robot_description"
        with NamedTemporaryFile(suffix=".rviz", mode="w", delete=False) as out_file:
            yaml.dump(config, out_file)
            return out_file.name
    else:
        # We do not need to do anything -- the template filename is the default for no namespace.
        return template_filename


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    hardware_interface: str = LaunchConfiguration("hardware_interface").perform(context)
    controllers_config: str = LaunchConfiguration("controllers_config").perform(context)
    mock_has_arm: bool = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)
    spot_name: str = LaunchConfiguration("spot_name").perform(context)

    # If connected to a physical robot, query if it has an arm. Otherwise, use the value in mock_has_arm.
    if hardware_interface == "robot":
        config_file = LaunchConfiguration("config_file").perform(context)
        has_arm = spot_has_arm(config_file_path=config_file, spot_name="")
        username, password, hostname = get_login_parameters(config_file)[:3]
        login_params = f" hostname:={hostname} username:={username} password:={password}"
    else:
        has_arm = mock_has_arm
        login_params = ""

    tf_prefix = f"{spot_name}/" if spot_name else ""

    # Generate the robot description based off if the robot has an arm.
    robot_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(THIS_PACKAGE), "xacro", "spot.urdf.xacro"]),
            " has_arm:=",
            str(has_arm),
            " tf_prefix:=",
            tf_prefix,
            " hardware_interface_type:=",
            LaunchConfiguration("hardware_interface"),
            login_params,
        ]
    )
    robot_description = {"robot_description": robot_urdf}

    # If no controller config file is selected, use the appropriate default. Else, just use the yaml that is passed in.
    if controllers_config == "":
        # Generate spot_default_controllers.yaml depending on namespace and whether the robot has an arm.
        create_controllers_config(spot_name, has_arm)
        controllers_config = create_controllers_config(spot_name, has_arm)

    # Add nodes
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controllers_config],
            namespace=spot_name,
        )
    )
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            namespace=spot_name,
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "controller_manager"],
            namespace=spot_name,
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[LaunchConfiguration("robot_controller"), "-c", "controller_manager"],
            namespace=spot_name,
        )
    )
    # Generate rviz configuration file based on the chosen namespace
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", create_rviz_config(spot_name)],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
            namespace=spot_name,
        )
    )
    return


def generate_launch_description():
    # Populate launch description with launch arguments
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_interface",
                default_value="mock",
                # Must match the xacro file options for which plugin to load
                choices=["mock", "robot"],
                description=(
                    "Hardware interface to load. 'mock' loads a simple interface useful for testing that forwards"
                    " commands directly to state. 'robot' uses a custom hardware interface using the Spot C++ SDK to"
                    " connect to the physical robot."
                ),
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Path to general configuration file optionally containing login information.",
            ),
            DeclareLaunchArgument(
                "controllers_config",
                default_value="",
                description=(
                    "Configuration file for the controllers loaded. If not set, a default config file containing a"
                    " forward position controller and a joint state publisher will be loaded, with the appropriate"
                    " configuration based on whether or not the robot has an arm."
                ),
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="forward_position_controller",
                choices=["forward_position_controller"],
                description="Robot controller to start. Must match an entry in controller_config.",
            ),
            DeclareLaunchArgument(
                "mock_has_arm",
                default_value="false",
                choices=["True", "true", "False", "false"],
                description="If in hardware_interface:=mock mode, whether or not the mocked robot has an arm.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="True",
                choices=["True", "true", "False", "false"],
                description="Flag to enable rviz.",
            ),
            DeclareLaunchArgument(
                "spot_name",
                default_value="",
                description="Name of the Spot that will be used as a namespace.",
            ),
        ]
    )
    # Add nodes to launch description
    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))
    return ld
