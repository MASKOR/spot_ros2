# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import yaml
from launch.actions import DeclareLaunchArgument

from spot_wrapper.wrapper import SpotWrapper

COLOR_END = "\33[0m"
COLOR_YELLOW = "\33[33m"

IMAGE_PUBLISHER_ARGS = [
    "depth_registered_mode",
    "publish_point_clouds",
    "uncompress_images",
    "publish_compressed_images",
    "stitch_front_images",
]


class DepthRegisteredMode(Enum):
    """Options for obtaining depth registered images images from Spot. We can request them from Spot's SDK, generate
    them using `depth_image_proc`'s nodelets, or just not publish them at all."""

    DISABLE = "disable"
    FROM_SPOT = "from_spot"
    FROM_NODELETS = "from_nodelets"

    def __repr__(self) -> str:
        return self.value


def declare_image_publisher_args() -> List[DeclareLaunchArgument]:
    """Generates launch arguments for each element in IMAGE_PUBLISHER_ARGS. This is useful to avoid copying and pasting
    the same launch arguments multiple times launchfiles that call the spot_image_publishers launchfile.

    Returns:
        List[DeclareLaunchArgument]: List of DeclareLaunchArguments useful for image publishing.
    """
    launch_args = []
    launch_args.append(
        DeclareLaunchArgument(
            "depth_registered_mode",
            default_value=DepthRegisteredMode.FROM_NODELETS.value,
            choices=[e.value for e in DepthRegisteredMode],
            description=(
                f"If `{DepthRegisteredMode.DISABLE.value}` is set, do not publish registered depth images."
                f" If `{DepthRegisteredMode.FROM_SPOT.value}` is set, request registered depth images from Spot through"
                f" its SDK. If `{DepthRegisteredMode.FROM_NODELETS.value}` is set, use depth_image_proc::RegisterNode"
                " component nodes running on the host computer to create registered depth images (this reduces the"
                " computational load on Spot's internal systems)."
            ),
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "publish_point_clouds",
            default_value="False",
            choices=["True", "true", "False", "false"],
            description=(
                "If true, create and publish point clouds for each depth registered and RGB camera pair. Requires that"
                " the depth_register_mode launch argument is set to a value that is not `disable`."
            ),
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "uncompress_images",
            default_value="True",
            choices=["True", "true", "False", "false"],
            description="Choose whether to publish uncompressed images from Spot.",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "publish_compressed_images",
            default_value="False",
            choices=["True", "true", "False", "false"],
            description="Choose whether to publish compressed images from Spot.",
        )
    )
    launch_args.append(
        DeclareLaunchArgument(
            "stitch_front_images",
            default_value="False",
            choices=["True", "true", "False", "false"],
            description=(
                "Choose whether to publish a stitched image constructed from Spot's front left and right cameras."
            ),
        )
    )
    return launch_args


def get_ros_param_dict(config_file_path: str) -> Dict[str, Any]:
    """Get a dictionary of parameter_name: parameter_value from a ROS config yaml file.

    Args:
        config_file_path (str): Path to the config yaml.

    Raises:
        YamlError: If the yaml can't be parsed
        ValueError: If the yaml file doesn't follow ROS conventions

    Returns:
        dict[str, Any]: dictionary of parameter_name: parameter_value.
        If there is no config file, returns the empty dictionary.
    """
    # an empty config file path is a valid way to start the driver, so here we just return the empty dictionary.
    if not config_file_path:
        return {}
    with open(config_file_path, "r") as config_yaml:
        try:
            config_dict = yaml.safe_load(config_yaml)
            if ("/**" in config_dict) and ("ros__parameters" in config_dict["/**"]):
                ros_params = config_dict["/**"]["ros__parameters"]
                return ros_params
            else:
                raise ValueError(
                    "Your yaml file does not follow ROS conventions! Make sure it starts with '/**' and"
                    " 'ros__parameters'."
                )
        except yaml.YAMLError as exc:
            raise yaml.YAMLError(f"Config file {config_file_path} couldn't be parsed: failed with '{exc}'")


def get_login_parameters(config_file_path: str) -> Tuple[str, str, str, Optional[int], Optional[str]]:
    """Obtain the username, password, hostname, port, and certificate of Spot from the environment variables or,
    if they are not set, the configuration file yaml.

    Args:
        config_file_path (str): Path to the configuration yaml

    Raises:
        ValueError: If any of username, password, hostname is not set.

    Returns:
        Tuple[str, str, str, Optional[int], Optional[str]]: username, password, hostname, port, certificate
    """
    # Get value from environment variables
    username = os.getenv("BOSDYN_CLIENT_USERNAME")
    password = os.getenv("BOSDYN_CLIENT_PASSWORD")
    hostname = os.getenv("SPOT_IP")
    portnum = os.getenv("SPOT_PORT")
    port = int(portnum) if portnum else None
    certificate = os.getenv("SPOT_CERTIFICATE")

    ros_params = get_ros_param_dict(config_file_path)
    # only set username/password/hostname if they were not already set as environment variables.
    if not username and "username" in ros_params:
        username = ros_params["username"]
    if not password and "password" in ros_params:
        password = ros_params["password"]
    if not hostname and "hostname" in ros_params:
        hostname = ros_params["hostname"]
    if not port and "port" in ros_params:
        port = ros_params["port"]
    if not certificate and "certificate" in ros_params:
        certificate = ros_params["certificate"]
    if (not username) or (not password) or (not hostname):
        raise ValueError(
            "One or more of your login credentials has not been specified! Got invalid values of "
            f"[Username: '{username}' Password: '{password}' Hostname: '{hostname}']. Ensure that your environment "
            "variables are set or update your config_file yaml."
        )
    return username, password, hostname, port, certificate


def default_camera_sources(has_arm: bool) -> List[str]:
    camera_sources = ["frontleft", "frontright", "left", "right", "back"]
    if has_arm:
        camera_sources.append("hand")
    return camera_sources


def get_camera_sources_from_ros_params(ros_params: Dict[str, Any], has_arm: bool) -> List[str]:
    """Get the list of cameras to stream from. This will be taken from the parameters in the config yaml if it exists
    and contains valid cameras. If this list contains invalid cameras, fall back to the default of all cameras.

    Args:
        ros_params (str): Dictionary of ros parameters from the config file.
        has_arm (bool): Whether or not your Spot has an arm.

    Raises:
        ValueError: If the parameter cameras_used is not formattted as a list.

    Returns:
        List[str]: List of cameras the driver will stream from.
    """
    default_sources = default_camera_sources(has_arm)
    if "cameras_used" in ros_params:
        camera_sources = ros_params["cameras_used"]
        if isinstance(camera_sources, List):
            # check if the user inputted any camera that's not in the default sources.
            invalid_cameras = [cam for cam in camera_sources if cam not in default_sources]
            if invalid_cameras:
                print(
                    f"{COLOR_YELLOW}WARNING: Your camera sources {camera_sources} contain invalid cameras. Make sure"
                    f" that the values are a subset of the default {default_sources}. Falling back to these default"
                    f" sources instead.{COLOR_END}"
                )
                return default_sources
            return camera_sources
        else:
            raise ValueError(f"Your camera sources {camera_sources} are not formatted correctly as a list!")
    else:
        return default_sources


def get_camera_sources(config_file_path: str, has_arm: bool) -> List[str]:
    """Wrapper around get_camera_sources_from_ros_params that grabs the ros parameters from the config file."""
    camera_sources = get_camera_sources_from_ros_params(get_ros_param_dict(config_file_path), has_arm)
    if len(camera_sources) == 0:
        print(f"{COLOR_YELLOW}WARNING: No camera sources are selected. Was this intended?{COLOR_END}")
    return camera_sources


def spot_has_arm(config_file_path: str, spot_name: str) -> bool:
    """Check if Spot has an arm querying the robot through SpotWrapper

    Args:
        config_file_path (str): Path to configuration yaml
        spot_name (str): Name of spot

    Returns:
        bool: True if spot has an arm, False otherwise
    """
    logger = logging.getLogger("spot_driver_launch")
    username, password, hostname, port, certificate = get_login_parameters(config_file_path)
    spot_wrapper = SpotWrapper(
        username=username,
        password=password,
        hostname=hostname,
        port=port,
        cert_resource_glob=certificate,
        robot_name=spot_name,
        logger=logger,
    )
    return spot_wrapper.has_arm()
