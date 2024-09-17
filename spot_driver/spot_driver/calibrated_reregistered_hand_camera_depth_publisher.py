#!/usr/bin/env python3

# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

"""
This script exists as it is not currently possible to set the intrinsic and extrinsic
parameters for the hand cameras through the bosdyn API. Currently, the registered
depth image is published using extrinsic and intrinsic parameters that are set under the hood. 
If the updated results from the calibration in spot_wrapper/spot_wrapper/calibration
can be populated into the underlying bosdyn API, then the default registered depth image
could be used without this script.

While it is possible to calibrate the gripper camera entirely under the hood, there is no way to
only set the intrinsic/extrinsic parameters (to preserve control over calibration parameters)

This script assumes you have a calibration generated with /spot_wrapper/spot_wrapper/calibration
"""
import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
import cv2
import numpy as np
import open3d as o3d
import yaml
from bdai_ros2_wrappers.context import wait_for_shutdown
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CalibratedReRegisteredHandCameraDepthPublisher:
    def __init__(
        self,
        calibration_path: Optional[str] = None,
        tag: str = "default",
        robot_name: Optional[str] = None,
        topic_name: str = "depth_registered/hand_custom_cal/image",
        undistort: bool = False,
        raw_depth_topic_suffix: Optional[str] = "/depth/hand/image",
    ):
        self.node = ros_scope.node()

        if calibration_path is None:
            self.node.get_logger().warning("No calibration path found, using default calibration")
            self.calibration = {
                "camera_matrix_depth": np.zeros(9).tolist(),
                "dist_coeffs_depth": np.zeros(5).tolist(),
                "camera_matrix_rgb": np.zeros(9).tolist(),
                "dist_coeffs_rgb": np.zeros(5).tolist(),
                "depth_t_rgb_R": np.zeros(9).tolist(),
                "depth_t_rgb_T": np.zeros(3).tolist(),
                "depth_image_dim": np.zeros(2).tolist(),
                "rgb_image_dim": np.zeros(2).tolist(),
            }
        else:
            self.calibration = extract_calibration_parameters(calibration_path=calibration_path, tag=tag)

        if robot_name is not None:
            self.calibration["robot_name"] = robot_name
        else:
            self.calibration["robot_name"] = ""
            self.node.get_logger().warning("No robot name supplied, assuming no namespace.")
        self.calibration["topic_name"] = topic_name
        self.calibration["undistort"] = undistort

        for param_name, param_val in self.calibration.items():
            self.node.get_logger().info(f"Setting the parameter {param_name} to a default value of {param_val}")
            self.node.declare_parameter(param_name, param_val)
            self.calibration[param_name] = self.node.get_parameter(param_name).value
            if param_name in ["camera_matrix_depth", "camera_matrix_rgb", "depth_t_rgb_R"]:
                self.calibration[param_name] = np.array(self.calibration[param_name]).reshape((3, 3))

            elif "dim" not in param_name and "name" not in param_name and "undistort" not in param_name:
                self.calibration[param_name] = np.array(self.calibration[param_name])

        if self.calibration["undistort"]:
            self.calculate_undistortion_parameters()

        self.cv_bridge = CvBridge()

        # by this point, if robot_name was not supplied or is None, robot_name is empty str
        reregistered_depth_topic = f"{self.calibration['robot_name']}/{topic_name}"
        raw_depth_topic = f"{self.calibration['robot_name']}{raw_depth_topic_suffix}"

        self.node.get_logger().info(f"Creating reregistered depth publisher to {reregistered_depth_topic}")
        self.reregistered_depth_img_pub = self.node.create_publisher(Image, reregistered_depth_topic, 10)

        self.node.get_logger().info(f"Creating subscriber to raw depth at {raw_depth_topic}")
        self.raw_depth_img_sub = self.node.create_subscription(
            Image, raw_depth_topic, self.republish_registered_depth_callback, 10
        )

    def republish_registered_depth_callback(self, msg: Image) -> None:
        if self.calibration["rgb_image_dim"][0] != 0:
            raw_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            pointcloud_pcd = self.depth_img_to_pointcloud(raw_depth_image, undistort=self.calibration["undistort"])
            reregistered_depth_image = self.pointcloud_to_depth_img(pointcloud_pcd)
            reregistered_depth_msg = msg
            reregistered_depth_msg.height = self.calibration["rgb_image_dim"][0]
            reregistered_depth_msg.width = self.calibration["rgb_image_dim"][1]
            reregistered_depth_msg.header.frame_id = f"{self.calibration['robot_name']}/hand_color_image_sensor"

            reregistered_depth_msg = self.cv_bridge.cv2_to_imgmsg(
                reregistered_depth_image, encoding="16UC1", header=reregistered_depth_msg.header
            )
            self.reregistered_depth_img_pub.publish(reregistered_depth_msg)
        else:
            self.node.get_logger().warning(
                "Cannot republish registered depth image without populating needed parameters."
            )

    def depth_img_to_pointcloud(
        self, depth_img: np.ndarray, undistort: bool = False, depth_scale: float = 1000.0, depth_max: float = 10
    ) -> o3d.t.geometry.PointCloud:
        depth_img = depth_img.astype(np.float32)

        if undistort:
            undistorted_depth_img = cv2.remap(
                depth_img,
                self.calibration["depth_image_undistort_map1"],
                self.calibration["depth_image_undistort_map2"],
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            undistorted_depth_img = depth_img

        intrinsic_tensor = o3d.core.Tensor(self.calibration["camera_matrix_depth"], dtype=o3d.core.Dtype.Float32)

        depth_tensor = o3d.core.Tensor(undistorted_depth_img, dtype=o3d.core.Dtype.Float32)
        depth_image = o3d.t.geometry.Image(depth_tensor)

        pointcloud_pcd = o3d.t.geometry.PointCloud.create_from_depth_image(
            depth=depth_image,
            intrinsics=intrinsic_tensor,
            depth_scale=depth_scale,
            depth_max=depth_max,
        )
        return pointcloud_pcd

    def pointcloud_to_depth_img(
        self, pointcloud: o3d.t.geometry.PointCloud, depth_scale: float = 1000.0, depth_max: float = 10
    ) -> np.ndarray:
        # Convert the extrinsic matrix from a NumPy array to an Open3D tensor
        extrinsic_np = np.eye(4)
        extrinsic_np[:3, :3] = self.calibration["depth_t_rgb_R"]
        extrinsic_np[:3, -1:] = self.calibration["depth_t_rgb_T"].reshape(3, 1)
        extrinsic_tensor = o3d.core.Tensor(extrinsic_np, dtype=o3d.core.Dtype.Float32)
        intrinsic_tensor = o3d.core.Tensor(self.calibration["camera_matrix_rgb"], dtype=o3d.core.Dtype.Float32)

        # Colors field needs to be populated to convert to depth image but doesn't matter for that
        # purpose so add random color
        if "colors" not in pointcloud.point:
            # Create a dummy color array with the same number of points, using all white (1, 1, 1)
            num_points = pointcloud.point["positions"].shape[0]
            dummy_colors = o3d.core.Tensor(np.ones((num_points, 3), dtype=np.float32), dtype=o3d.core.Dtype.Float32)
            pointcloud.point["colors"] = dummy_colors
        rgbd_image = pointcloud.project_to_rgbd_image(
            width=self.calibration["rgb_image_dim"][1],
            height=self.calibration["rgb_image_dim"][0],
            intrinsics=intrinsic_tensor,
            extrinsics=extrinsic_tensor,
            depth_scale=depth_scale,
            depth_max=depth_max,
        )
        depth_image = np.asarray(rgbd_image.depth)
        return depth_image.astype(np.uint16)

    def calculate_undistortion_parameters(self) -> None:
        h, w = self.calibration["depth_image_dim"]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.calibration["camera_matrix_depth"], self.calibration["dist_coeffs_depth"], (w, h), 1, (w, h)
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            self.calibration["camera_matrix_depth"],
            self.calibration["dist_coeffs_depth"],
            None,
            new_camera_matrix,
            (w, h),
            cv2.CV_32FC1,
        )
        self.calibration["depth_image_undistort_map1"] = map1
        self.calibration["depth_image_undistort_map2"] = map2
        self.calibration["camera_matrix_depth"] = new_camera_matrix


def extract_calibration_parameters(calibration_path: str, tag: str) -> dict:
    try:
        with open(calibration_path, "r") as file:
            recorded_calibration = yaml.safe_load(file)
    except FileNotFoundError:
        raise ValueError(f"Error: The calibration file at {calibration_path} was not found.")
    except yaml.YAMLError as e:
        raise ValueError(f"Error: There was an issue parsing the calibration YAML file: {e}")
    try:
        calibration = {}
        calibration["camera_matrix_depth"] = recorded_calibration[tag]["intrinsic"][1]["camera_matrix"]
        calibration["dist_coeffs_depth"] = recorded_calibration[tag]["intrinsic"][1]["dist_coeffs"]
        calibration["camera_matrix_rgb"] = recorded_calibration[tag]["intrinsic"][0]["camera_matrix"]
        calibration["dist_coeffs_rgb"] = recorded_calibration[tag]["intrinsic"][1]["dist_coeffs"]
        calibration["depth_t_rgb_R"] = recorded_calibration[tag]["extrinsic"][1][0]["R"]
        calibration["depth_t_rgb_T"] = recorded_calibration[tag]["extrinsic"][1][0]["T"]
        calibration["depth_image_dim"] = recorded_calibration[tag]["intrinsic"][1]["image_dim"]
        calibration["rgb_image_dim"] = recorded_calibration[tag]["intrinsic"][0]["image_dim"]
    except KeyError as e:
        raise ValueError(f"Error: Missing key in the calibration data: {e}")
    except TypeError as e:
        raise ValueError(f"Error: Incorrect data type or structure in the calibration data: {e}")
    except ValueError as e:
        raise ValueError(f"Error: Invalid value in calibration data: {e}")

    return calibration


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibration_path",
        "-c",
        "--cal",
        "--calibration",
        dest="calibration_path",
        required=False,
        type=str,
        help="A calibration saved with the multi-stereo charuco calibration utility",
    )
    parser.add_argument(
        "--robot_name",
        dest="robot_name",
        default=None,
        required=False,
        type=str,
        help="Spot Robot namespace. Don't supply this argument if there's no namespace.",
    )
    parser.add_argument(
        "--tag", dest="tag", default="default", type=str, help="What tag to load from the calibration file"
    )
    parser.add_argument(
        "--topic",
        dest="topic",
        default="depth_registered/hand_custom_cal/image",
        type=str,
        help="what topic suffix to publish the reregistered image at. No leading slash!",
    )
    parser.add_argument(
        "--undistort",
        dest="undistort",
        default=False,
        type=bool,
        help="Whether to undistort the depth image prior to republishing it.",
    )
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    CalibratedReRegisteredHandCameraDepthPublisher(
        calibration_path=args.calibration_path,
        tag=args.tag,
        robot_name=args.robot_name,
        topic_name=args.topic,
        undistort=args.undistort,
    )
    wait_for_shutdown()


if __name__ == "__main__":
    main()
