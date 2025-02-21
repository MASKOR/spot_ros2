# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

import os
from typing import List

import launch
import launch_ros
from launch import LaunchContext, LaunchDescription, Substitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from spot_driver.launch.spot_launch_helpers import (
    DepthRegisteredMode,
    declare_image_publisher_args,
    get_camera_sources,
    get_name_and_prefix,
    spot_has_arm,
    substitute_launch_parameters,
)


def create_depth_registration_nodelets(
    context: launch.LaunchContext,
    spot_name: str,
    camera_sources: List[str],
) -> List[launch_ros.descriptions.ComposableNode]:
    """Create the list of depth_image_proc::RegisterNode composable nodes required to generate registered depth images
    for Spot's cameras."""

    composable_node_descriptions = []

    for camera in camera_sources:
        composable_node_descriptions.append(
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node_" + camera,
                namespace=spot_name,
                # Each entry in the remappings list is a tuple.
                # The first element in the tuple is the internal name of the topic used within the nodelet.
                # The second element is the external name of the topic used by other nodes in the system.
                remappings=[
                    ("depth/image_rect", PathJoinSubstitution(["depth", camera, "image"]).perform(context)),
                    ("depth/camera_info", PathJoinSubstitution(["depth", camera, "camera_info"]).perform(context)),
                    ("rgb/camera_info", PathJoinSubstitution(["camera", camera, "camera_info"]).perform(context)),
                    (
                        "depth_registered/image_rect",
                        PathJoinSubstitution(["depth_registered", camera, "image"]).perform(context),
                    ),
                    (
                        "depth_registered/camera_info",
                        PathJoinSubstitution(["depth_registered", camera, "camera_info"]).perform(context),
                    ),
                ],
            )
        )
    return composable_node_descriptions


def create_point_cloud_nodelets(
    context: launch.LaunchContext,
    spot_name: str,
    camera_sources: List[str],
) -> List[launch_ros.descriptions.ComposableNode]:
    """Create the list of depth_image_proc::PointCloudXyzrgbNode composable nodes required to generate point clouds for
    each pair of RGB and registered depth cameras."""

    composable_node_descriptions = []

    for camera in camera_sources:
        composable_node_descriptions.append(
            launch_ros.descriptions.ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb_node_" + camera,
                namespace=spot_name,
                # Each entry in the remappings list is a tuple.
                # The first element in the tuple is the internal name of the topic used within the nodelet.
                # The second element is the external name of the topic used by other nodes in the system.
                remappings=[
                    ("rgb/camera_info", PathJoinSubstitution(["camera", camera, "camera_info"]).perform(context)),
                    ("rgb/image_rect_color", PathJoinSubstitution(["camera", camera, "image"]).perform(context)),
                    (
                        "depth_registered/image_rect",
                        PathJoinSubstitution(["depth_registered", camera, "image"]).perform(context),
                    ),
                    ("points", PathJoinSubstitution(["depth_registered", camera, "points"]).perform(context)),
                ],
            ),
        )
    return composable_node_descriptions


def launch_setup(context: LaunchContext, ld: LaunchDescription) -> None:
    config_file = LaunchConfiguration("config_file")
    spot_name_arg = LaunchConfiguration("spot_name")
    tf_prefix_arg = LaunchConfiguration("tf_prefix")
    depth_registered_mode_config = LaunchConfiguration("depth_registered_mode")
    publish_point_clouds_config = LaunchConfiguration("publish_point_clouds")
    mock_enable = IfCondition(LaunchConfiguration("mock_enable", default="False")).evaluate(context)

    # if config_file has been set (and is not the default empty string) and is also not a file, do not launch anything.
    config_file_path = config_file.perform(context)
    if (config_file_path != "") and (not os.path.isfile(config_file_path)):
        raise FileNotFoundError("Configuration file '{}' does not exist!".format(config_file_path))

    substitutions = {
        "spot_name": spot_name_arg,
        "frame_prefix": tf_prefix_arg,
    }
    configured_params = substitute_launch_parameters(config_file, substitutions, context)
    spot_name, _ = get_name_and_prefix(configured_params)
    if isinstance(spot_name, Substitution):
        spot_name = spot_name.perform(context)

    if mock_enable:
        mock_has_arm = IfCondition(LaunchConfiguration("mock_has_arm")).evaluate(context)
        has_arm = mock_has_arm
    else:
        has_arm = spot_has_arm(config_file_path=config_file.perform(context))

    camera_sources = get_camera_sources(config_file_path, has_arm)

    depth_registered_mode_string = depth_registered_mode_config.perform(context).lower()
    depth_registered_mode = DepthRegisteredMode(depth_registered_mode_string)

    publish_point_clouds = True if publish_point_clouds_config.perform(context).lower() == "true" else False
    if depth_registered_mode is DepthRegisteredMode.DISABLE and publish_point_clouds:
        print(
            "Warning: Point cloud publisher nodelets will not be launched because depth_registered_mode is set to"
            " `disable`. Set depth_registered_mode to `from_nodelets` or `from_spot` to enable point cloud publishing."
        )
        publish_point_clouds = False

    spot_image_publisher_params = {
        key: LaunchConfiguration(key) for key in ["uncompress_images", "publish_compressed_images"]
    }

    # If using nodelets to generate registered depth images, do not retrieve and publish registered depth images using
    # spot_image_publisher_node.
    if depth_registered_mode is not DepthRegisteredMode.FROM_SPOT:
        spot_image_publisher_params.update({"publish_depth_registered": False})

    spot_image_publisher_node = launch_ros.actions.Node(
        package="spot_driver",
        executable="spot_image_publisher_node",
        output="screen",
        parameters=[configured_params, spot_image_publisher_params],
        namespace=spot_name,
    )
    ld.add_action(spot_image_publisher_node)

    # Parse config options to create a list of composable node descriptions for the nodelets we want to run within the
    # composable node container.
    composable_node_descriptions = (
        create_depth_registration_nodelets(context, spot_name, camera_sources)
        if depth_registered_mode is DepthRegisteredMode.FROM_NODELETS
        else []
    ) + (create_point_cloud_nodelets(context, spot_name, camera_sources) if publish_point_clouds else [])
    container = launch_ros.actions.ComposableNodeContainer(
        name="container",
        namespace=spot_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=composable_node_descriptions,
    )
    ld.add_action(container)

    # add the image stitcher node, but only if frontleft and frontright cameras are enabled.
    if "frontleft" in camera_sources and "frontright" in camera_sources:
        virtual_camera_frame = "frontmiddle_virtual"
        stitcher_params = {
            "body_frame": "body",
            "virtual_camera_frame": virtual_camera_frame,
        }
        cam_prefix = f"/{spot_name}" if spot_name else ""
        image_stitcher_node = launch_ros.actions.Node(
            package="spot_driver",
            executable="image_stitcher_node",
            namespace=spot_name,
            output="screen",
            remappings=[
                (f"{cam_prefix}/left/image", f"{cam_prefix}/camera/frontleft/image"),
                (f"{cam_prefix}/left/camera_info", f"{cam_prefix}/camera/frontleft/camera_info"),
                (f"{cam_prefix}/right/image", f"{cam_prefix}/camera/frontright/image"),
                (f"{cam_prefix}/right/camera_info", f"{cam_prefix}/camera/frontright/camera_info"),
                (f"{cam_prefix}/virtual_camera/image", f"{cam_prefix}/camera/{virtual_camera_frame}/image"),
                (f"{cam_prefix}/virtual_camera/camera_info", f"{cam_prefix}/camera/{virtual_camera_frame}/camera_info"),
            ],
            parameters=[configured_params, stitcher_params],
            condition=IfCondition(LaunchConfiguration("stitch_front_images")),
        )
        ld.add_action(image_stitcher_node)


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
    launch_args.append(DeclareLaunchArgument("spot_name", default_value="", description="Name of Spot"))
    launch_args += declare_image_publisher_args()

    ld = launch.LaunchDescription(launch_args)

    ld.add_action(OpaqueFunction(function=launch_setup, args=[ld]))

    return ld
