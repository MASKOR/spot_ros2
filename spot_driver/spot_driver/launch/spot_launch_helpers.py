# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
from enum import Enum
from typing import Any, Dict, List, Literal, Optional, Tuple, Union

import yaml
from launch import LaunchContext, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from synchros2.launch.actions import DeclareBooleanLaunchArgument

from spot_wrapper.wrapper import SpotWrapper

COLOR_END = "\33[0m"
COLOR_YELLOW = "\33[33m"

# The following constants are parameters we are interested in pulling from our config yaml file
_USERNAME: Literal["username"] = "username"
_PASSWORD: Literal["password"] = "password"
_HOSTNAME: Literal["hostname"] = "hostname"
_CERTIFICATE: Literal["certificate"] = "certificate"
_PORT: Literal["port"] = "port"
_CAMERAS_USED: Literal["cameras_used"] = "cameras_used"
_GRIPPERLESS: Literal["gripperless"] = "gripperless"
_SPOT_NAME: Literal["spot_name"] = "spot_name"


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
        DeclareBooleanLaunchArgument(
            "publish_point_clouds",
            default_value=False,
            description=(
                "If true, create and publish point clouds for each depth registered and RGB camera pair. Requires that"
                " the depth_register_mode launch argument is set to a value that is not `disable`."
            ),
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "uncompress_images",
            default_value=True,
            description="Choose whether to publish uncompressed images from Spot.",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "publish_compressed_images",
            default_value=False,
            description="Choose whether to publish compressed images from Spot.",
        )
    )
    launch_args.append(
        DeclareBooleanLaunchArgument(
            "stitch_front_images",
            default_value=False,
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


def get_login_parameters(config_file_path: str) -> Tuple[str, str, str, Optional[int], Optional[str], Optional[str]]:
    """Obtain the username, password, hostname, port, certificate, and name of Spot from the environment variables or,
    if they are not set, the configuration file yaml.

    Args:
        config_file_path (str): Path to the configuration yaml

    Raises:
        ValueError: If any of username, password, hostname is not set.

    Returns:
        Tuple[str, str, str, Optional[int], Optional[str], Optional[str]]: username, password, hostname, port,
        certificate, spot_name
    """
    # Get value from environment variables
    username = os.getenv("BOSDYN_CLIENT_USERNAME")
    password = os.getenv("BOSDYN_CLIENT_PASSWORD")
    hostname = os.getenv("SPOT_IP")
    portnum = os.getenv("SPOT_PORT")
    port = int(portnum) if portnum else None
    certificate = os.getenv("SPOT_CERTIFICATE")
    spot_name: Optional[str] = None

    ros_params = get_ros_param_dict(config_file_path)
    # only set username/password/hostname if they were not already set as environment variables.
    if not username and _USERNAME in ros_params:
        username = ros_params[_USERNAME]
    if not password and _PASSWORD in ros_params:
        password = ros_params[_PASSWORD]
    if not hostname and _HOSTNAME in ros_params:
        hostname = ros_params[_HOSTNAME]
    if not port and _PORT in ros_params:
        port = ros_params[_PORT]
    if not certificate and _CERTIFICATE in ros_params:
        certificate = ros_params[_CERTIFICATE]
    if (not username) or (not password) or (not hostname):
        raise ValueError(
            "One or more of your login credentials has not been specified! Got invalid values of "
            f"[Username: '{username}' Password: '{password}' Hostname: '{hostname}']. Ensure that your environment "
            "variables are set or update your config_file yaml."
        )
    if _SPOT_NAME in ros_params and ros_params[_SPOT_NAME]:
        spot_name = ros_params[_SPOT_NAME]
    return username, password, hostname, port, certificate, spot_name


def default_camera_sources(has_arm: bool, gripperless: bool) -> List[str]:
    camera_sources = ["frontleft", "frontright", "left", "right", "back"]
    if has_arm and not gripperless:
        camera_sources.append("hand")
    return camera_sources


def get_gripperless(ros_params: Dict[str, Any]) -> bool:
    """Read the ros parameters to get the value of the gripperless parameter. Defaults to False if not set."""
    gripperless = False
    if _GRIPPERLESS in ros_params:
        if isinstance(ros_params[_GRIPPERLESS], bool):
            gripperless = ros_params[_GRIPPERLESS]
    return gripperless


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
    gripperless = get_gripperless(ros_params)
    default_sources = default_camera_sources(has_arm, gripperless)
    if _CAMERAS_USED in ros_params:
        camera_sources = ros_params[_CAMERAS_USED]
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


def spot_has_arm(config_file_path: str) -> bool:
    """Check if Spot has an arm querying the robot through SpotWrapper

    Args:
        config_file_path (str): Path to configuration yaml

    Returns:
        bool: True if spot has an arm, False otherwise
    """
    logger = logging.getLogger("spot_driver_launch")
    username, password, hostname, port, certificate, spot_name = get_login_parameters(config_file_path)
    gripperless = get_gripperless(get_ros_param_dict(config_file_path))
    spot_wrapper = SpotWrapper(
        username=username,
        password=password,
        hostname=hostname,
        port=port,
        cert_resource_glob=certificate,
        robot_name=spot_name,
        logger=logger,
        gripperless=gripperless,
    )
    return spot_wrapper.has_arm()


def substitute_launch_parameters(
    config_file_path: Union[str, Substitution],
    substitutions: Dict[str, Substitution],
    context: LaunchContext,
) -> Dict[str, Any]:
    """Pass the given ROS launch parameter substitutions into parameters from the ROS config yaml file.

    Args:
        config_file_path (str | Substitution): Path to the config yaml.
        substitutions (Dict[str, Substitution]): Dictionary of parameter_name: parameter_value containing the desired
                                                 launch parameter substitutions.
        context (LaunchContext): Context for acquiring the launch configuration inner values.

    Returns:
        dict[str, Any]: dictionary of the substituted parameter_name: parameter_value.
        If there is no config file, returns a dictionary containing only the given substitutions.
        If there is no config file and the substitutions don't have any values, returns an empty dictionary.
    """
    config_params: Dict[str, Any] = get_ros_param_dict(
        config_file_path if isinstance(config_file_path, str) else config_file_path.perform(context)
    )
    for key, value in substitutions.items():
        if value.perform(context):
            config_params[key] = value

    return config_params


def get_name_and_prefix(ros_params: Dict[str, Any]) -> Tuple[Union[str, Substitution], Union[str, Substitution]]:
    """Get the Spot robot name and ROS TF frame prefix from the provided ROS parameters, which may be taken directly
    from the yaml config or passed through from launch arguments. This will compose the frame prefix from the Spot name
    if not given explicitly.

    Args:
        ros_params (dict[str, Any]): A dictionary of parameter_name: parameter_value.

    Returns:
        Tuple[str | Substitution, str | Substitution]: spot_name, tf_prefix.
    """
    spot_name: Union[str, Substitution] = ros_params["spot_name"] if "spot_name" in ros_params else ""
    tf_prefix: Optional[Union[str, Substitution]] = ros_params["frame_prefix"] if "frame_prefix" in ros_params else None
    if tf_prefix is None:
        if isinstance(spot_name, Substitution):
            tf_prefix = PathJoinSubstitution([spot_name, ""])
        elif isinstance(spot_name, str) and spot_name:
            tf_prefix = spot_name + "/"
        else:
            tf_prefix = ""

    return spot_name, tf_prefix
