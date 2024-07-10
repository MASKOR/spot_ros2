#!/usr/bin/env python3
# Debug
# from ros_helpers import *
import logging
import os
import tempfile
import threading
import time
import traceback
import typing
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Union

import bdai_ros2_wrappers.process as ros_process
import builtin_interfaces.msg
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.single_goal_action_server import (
    SingleGoalActionServer,
)
from bdai_ros2_wrappers.single_goal_multiple_action_servers import (
    SingleGoalMultipleActionServers,
)
from bosdyn.api import (
    geometry_pb2,
    gripper_camera_param_pb2,
    manipulation_api_pb2,
    robot_command_pb2,
    trajectory_pb2,
    world_object_pb2,
)
from bosdyn.api.geometry_pb2 import Quaternion, SE2VelocityLimit
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.spot.choreography_sequence_pb2 import Animation, ChoreographySequence, ChoreographyStatusResponse
from bosdyn.client import math_helpers
from bosdyn.client.exceptions import InternalServerError
from bosdyn_api_msgs.math_helpers import bosdyn_localization_to_pose_msg
from bosdyn_msgs.conversions import convert
from bosdyn_msgs.msg import (
    ArmCommandFeedback,
    Camera,
    FullBodyCommandFeedback,
    GripperCommandFeedback,
    Logpoint,
    ManipulationApiFeedbackResponse,
    MobilityCommandFeedback,
    PtzDescription,
    RobotCommandFeedback,
    RobotCommandFeedbackStatusStatus,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Twist,
)
from rclpy import Parameter
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.impl import rcutils_logger
from rclpy.publisher import Publisher
from rclpy.timer import Rate
from std_srvs.srv import SetBool, Trigger

import spot_driver.robot_command_util as robot_command_util

# DEBUG/RELEASE: RELATIVE PATH NOT WORKING IN DEBUG
# Release
from spot_driver.ros_helpers import (
    get_from_env_and_fall_back_to_param,
)
from spot_msgs.action import (  # type: ignore
    ExecuteDance,
    Manipulation,
    NavigateTo,
    Trajectory,
)
from spot_msgs.action import (  # type: ignore
    RobotCommand as RobotCommandAction,
)
from spot_msgs.msg import (  # type: ignore
    Feedback,
    LeaseArray,
    LeaseResource,
    Metrics,
    MobilityParams,
)
from spot_msgs.srv import (  # type: ignore
    ChoreographyRecordedStateToAnimation,
    ChoreographyStartRecordingState,
    ChoreographyStopRecordingState,
    ClearBehaviorFault,
    DeleteLogpoint,
    DeleteSound,
    Dock,
    GetChoreographyStatus,
    GetGripperCameraParameters,
    GetLEDBrightness,
    GetLogpointStatus,
    GetPtzPosition,
    GetVolume,
    GraphNavClearGraph,
    GraphNavGetLocalizationPose,
    GraphNavSetLocalization,
    GraphNavUploadGraph,
    InitializeLens,
    ListAllDances,
    ListAllMoves,
    ListCameras,
    ListGraph,
    ListLogpoints,
    ListPtz,
    ListSounds,
    ListWorldObjects,
    LoadSound,
    OverrideGraspOrCarry,
    PlaySound,
    RetrieveLogpoint,
    SetGripperCameraParameters,
    SetLEDBrightness,
    SetLocomotion,
    SetPtzPosition,
    SetVelocity,
    SetVolume,
    StoreLogpoint,
    TagLogpoint,
    UploadAnimation,
    UploadSequence,
)
from spot_msgs.srv import (  # type: ignore
    RobotCommand as RobotCommandService,
)
from spot_wrapper.cam_wrapper import SpotCamCamera, SpotCamWrapper
from spot_wrapper.wrapper import SpotWrapper

MAX_DURATION = 1e6
COLOR_END = "\33[0m"
COLOR_GREEN = "\33[32m"
COLOR_YELLOW = "\33[33m"


@dataclass
class Request:
    id: str
    data: Any


@dataclass
class Response:
    message: str
    success: bool


class GoalResponse(Enum):
    FAILED = -1
    IN_PROGRESS = False
    SUCCESS = True


class WaitForGoal(object):
    def __init__(
        self,
        clock: Clock,
        _time: Union[float, rclpy.time.Time],
        callback: Optional[Callable] = None,
    ) -> None:
        self._at_goal: bool = False
        self._callback: Optional[Callable] = callback
        self._clock: Clock = clock
        self._time: rclpy.time.Duration = rclpy.time.Duration(seconds=_time)
        self._thread: threading.Thread = threading.Thread(target=self._run)
        self._thread.start()

    @property
    def at_goal(self) -> bool:
        return self._at_goal

    def _run(self) -> None:
        start_time = self._clock.now()
        while self._clock.now() - start_time < self._time:
            time.sleep(0.05)
        self._at_goal = True


def set_node_parameter_from_parameter_list(
    node: Node, parameter_list: Optional[typing.List[Parameter]], parameter_name: str
) -> None:
    """Set parameters when the node starts not from a launch file."""
    if parameter_list is not None:
        node.set_parameters([parameter for parameter in parameter_list if parameter.name == parameter_name])


class SpotROS(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    TRAJECTORY_BATCH_SIZE_PARAM = "trajectory_batch_size"
    TRAJECTORY_BATCH_OVERLAPPING_POINTS_PARAM = "trajectory_batch_overlapping_points"

    def __init__(self, parameter_list: Optional[typing.List[Parameter]] = None, **kwargs: typing.Any) -> None:
        """
        Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.
        Holds lease from wrapper and updates all async tasks at the ROS rate
        """
        super().__init__("spot_ros2", **kwargs)
        self.run_navigate_to: Optional[bool] = None
        self._printed_once: bool = False

        self.get_logger().info(COLOR_GREEN + "Hi from spot_driver." + COLOR_END)

        self.callbacks: Dict[str, Callable] = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["metrics"] = self.metrics_callback
        self.callbacks["lease"] = self.lease_callback

        self.group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        self.graph_nav_callback_group: CallbackGroup = MutuallyExclusiveCallbackGroup()
        rate = self.create_rate(100)
        self.node_rate: Rate = rate

        self.declare_parameter("auto_claim", False)
        self.declare_parameter("auto_power_on", False)
        self.declare_parameter("auto_stand", False)

        self.declare_parameter("use_take_lease", True)
        self.declare_parameter("get_lease_on_action", True)
        self.declare_parameter("continually_try_stand", False)

        self.declare_parameter("estop_timeout", 9.0)
        self.declare_parameter("cmd_duration", 0.125)
        self.declare_parameter("start_estop", False)
        self.declare_parameter("rgb_cameras", True)

        # Declare rates for the spot_ros2 publishers, which are combined to a dictionary
        self.declare_parameter("metrics_rate", 0.04)
        self.declare_parameter("lease_rate", 1.0)
        self.declare_parameter("world_objects_rate", 20.0)
        self.declare_parameter("graph_nav_pose_rate", 10.0)

        self.declare_parameter("publish_graph_nav_pose", False)
        self.declare_parameter("graph_nav_seed_frame", "graph_nav_map")
        self.declare_parameter("initialize_spot_cam", False)

        self.declare_parameter("spot_name", "")
        self.declare_parameter("mock_enable", False)

        # When we send very long trajectories to Spot, we create batches of
        # given size. If we do not batch a long trajectory, Spot will reject it.
        self.declare_parameter(self.TRAJECTORY_BATCH_SIZE_PARAM, 100)
        self.trajectory_batch_size: Parameter = self.get_parameter(self.TRAJECTORY_BATCH_SIZE_PARAM).value

        # When we send very long trajectories to Spot, we create overlapping
        # batches. Overlapping trajectories is very important because we want
        # the robot to stitch them smoothly. A batch is sent before the
        # previous one has completed, to work around network latency. The
        # following default value has been determined empirically.
        self.declare_parameter(self.TRAJECTORY_BATCH_OVERLAPPING_POINTS_PARAM, 20)
        self.trajectory_batch_overlapping_points: Parameter = self.get_parameter(
            self.TRAJECTORY_BATCH_OVERLAPPING_POINTS_PARAM
        ).value

        # If `mock_enable:=True`, then there are additional parameters. We must set this one separately.
        set_node_parameter_from_parameter_list(self, parameter_list, "mock_enable")
        if self.get_parameter("mock_enable").value:
            self.declare_parameter("mock_has_arm", rclpy.Parameter.Type.BOOL)

        # used for setting when not using launch file
        if parameter_list is not None:
            self.set_parameters(parameter_list)

        self.auto_claim: Parameter = self.get_parameter("auto_claim")
        self.auto_power_on: Parameter = self.get_parameter("auto_power_on")
        self.auto_stand: Parameter = self.get_parameter("auto_stand")
        self.start_estop: Parameter = self.get_parameter("start_estop")

        self.use_take_lease: Parameter = self.get_parameter("use_take_lease")
        self.get_lease_on_action: Parameter = self.get_parameter("get_lease_on_action")
        self.continually_try_stand: Parameter = self.get_parameter("continually_try_stand")

        self.rgb_cameras: Parameter = self.get_parameter("rgb_cameras")

        self.publish_graph_nav_pose: Parameter = self.get_parameter("publish_graph_nav_pose")
        self.graph_nav_seed_frame: str = self.get_parameter("graph_nav_seed_frame").value
        self.initialize_spot_cam: bool = self.get_parameter("initialize_spot_cam").value

        self._wait_for_goal: Optional[WaitForGoal] = None
        self.goal_handle: Optional[ServerGoalHandle] = None

        self.rates = {
            "metrics": self.get_parameter("metrics_rate").value,
            "lease": self.get_parameter("lease_rate").value,
            "world_objects": self.get_parameter("world_objects_rate").value,
            "graph_nav_pose": self.get_parameter("graph_nav_pose_rate").value,
        }
        max_task_rate = float(max(self.rates.values()))

        self.declare_parameter("async_tasks_rate", max_task_rate)
        # This is only done from parameter because it should be passed by the launch file
        self.name: Optional[str] = self.get_parameter("spot_name").value
        if not self.name:
            self.name = None
        self.mock: bool = self.get_parameter("mock_enable").value
        self.mock_has_arm: Optional[bool] = None
        if self.mock:
            self.mock_has_arm = self.get_parameter("mock_has_arm").value

        self.estop_timeout: Parameter = self.get_parameter("estop_timeout")
        self.async_tasks_rate: float = self.get_parameter("async_tasks_rate").value
        if self.async_tasks_rate < max_task_rate:
            self.get_logger().warn(
                COLOR_YELLOW
                + f"The maximum individual task rate is {max_task_rate} Hz. You have manually set the async_tasks_rate"
                f" to {self.async_tasks_rate} which is lower and will decrease the frequency of one of the periodic"
                " tasks being run."
                + COLOR_END
            )

        self.cmd_duration: float = self.get_parameter("cmd_duration").value

        self.username: str = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", self, "username", "user")
        self.password: str = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", self, "password", "password")

        self.ip: str = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")
        self.port: int = get_from_env_and_fall_back_to_param("SPOT_PORT", self, "port", 0)
        self.certificate: Optional[str] = (
            get_from_env_and_fall_back_to_param("SPOT_CERTIFICATE", self, "certificate", "") or None
        )

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        # These params enables to change which odometry frame is a parent of body frame and to change tf names of each
        # odometry frames.
        frame_prefix = ""
        if self.name is not None:
            frame_prefix = self.name + "/"
        self.frame_prefix: str = frame_prefix
        self.preferred_odom_frame: Parameter = self.declare_parameter(
            "preferred_odom_frame", self.frame_prefix + "odom"
        )  # 'vision' or 'odom'
        self.tf_name_kinematic_odom: Parameter = self.declare_parameter(
            "tf_name_kinematic_odom", self.frame_prefix + "odom"
        )
        self.tf_name_raw_kinematic: str = frame_prefix + "odom"
        self.tf_name_vision_odom: Parameter = self.declare_parameter(
            "tf_name_vision_odom", self.frame_prefix + "vision"
        )
        self.tf_name_raw_vision: str = self.frame_prefix + "vision"

        preferred_odom_frame_references = [self.tf_name_raw_kinematic, self.tf_name_raw_vision]
        if self.preferred_odom_frame.value not in preferred_odom_frame_references:
            error_msg = (
                f'rosparam "preferred_odom_frame" should be one of {preferred_odom_frame_references}, got'
                f' "{self.preferred_odom_frame.value}"'
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)

        self.tf_name_graph_nav_body: str = self.frame_prefix + "body"

        # logger for spot wrapper
        name_with_dot = ""
        if self.name is not None:
            name_with_dot = self.name + "."

        logging.basicConfig(format="[%(filename)s:%(lineno)d] %(message)s", level=logging.ERROR)
        self.wrapper_logger = logging.getLogger(f"{name_with_dot}spot_wrapper")

        name_str = ""
        if self.name is not None:
            name_str = " for " + self.name
        mocking_designator = " (mocked)" if self.mock else ""
        self.get_logger().info("Starting ROS driver for Spot" + name_str + mocking_designator)
        # testing with Robot

        if self.mock:
            self.spot_wrapper: Optional[SpotWrapper] = None
            self.cam_wrapper: Optional[SpotCamWrapper] = None
        else:
            # create SpotWrapper if not mocking
            self.spot_wrapper = SpotWrapper(
                username=self.username,
                password=self.password,
                hostname=self.ip,
                port=self.port,
                robot_name=self.name,
                logger=self.wrapper_logger,
                start_estop=self.start_estop.value,
                estop_timeout=self.estop_timeout.value,
                rates=self.rates,
                callbacks=self.callbacks,
                use_take_lease=self.use_take_lease.value,
                get_lease_on_action=self.get_lease_on_action.value,
                continually_try_stand=self.continually_try_stand.value,
                rgb_cameras=self.rgb_cameras.value,
                cert_resource_glob=self.certificate,
            )
            if not self.spot_wrapper.is_valid:
                return

            self.spot_cam_wrapper = None
            if self.initialize_spot_cam:
                try:
                    self.cam_logger = rcutils_logger.RcutilsLogger(name=f"{name_with_dot}spot_cam_wrapper")
                    self.spot_cam_wrapper = SpotCamWrapper(
                        hostname=self.ip,
                        username=self.username,
                        password=self.password,
                        port=self.port,
                        logger=self.cam_logger,
                        cert_resource_glob=self.certificate,
                    )
                except SystemError:
                    self.spot_cam_wrapper = None

            if self.frame_prefix != self.spot_wrapper.frame_prefix:
                error_msg = (
                    f"ERROR: disagreement between `self.frame_prefix` ({self.frame_prefix}) and"
                    f" `self.spot_wrapper.frame_prefix` ({self.spot_wrapper.frame_prefix})"
                )
                self.get_logger().error(error_msg)
                raise ValueError(error_msg)

        all_cameras = ["frontleft", "frontright", "left", "right", "back"]
        has_arm = self.mock_has_arm
        if self.spot_wrapper is not None:
            has_arm = self.spot_wrapper.has_arm()
        if has_arm:
            all_cameras.append("hand")
        self.declare_parameter("cameras_used", all_cameras)
        self.cameras_used = self.get_parameter("cameras_used")

        if self.publish_graph_nav_pose.value:
            # graph nav pose will be published both on a topic
            # and as a TF transform from graph_nav_map to body.
            self.graph_nav_pose_pub = self.create_publisher(PoseStamped, "graph_nav/body_pose", 1)
            self.graph_nav_pose_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            self.create_timer(
                1 / self.rates["graph_nav_pose"],
                self.publish_graph_nav_pose_callback,
                callback_group=self.graph_nav_callback_group,
            )

        self.declare_parameter("has_arm", has_arm)

        # Status Publishers #
        self.dynamic_broadcaster: tf2_ros.TransformBroadcaster = tf2_ros.TransformBroadcaster(self)
        self.metrics_pub: Publisher = self.create_publisher(Metrics, "status/metrics", 1)
        self.lease_pub: Publisher = self.create_publisher(LeaseArray, "status/leases", 1)
        self.feedback_pub: Publisher = self.create_publisher(Feedback, "status/feedback", 1)
        self.mobility_params_pub: Publisher = self.create_publisher(MobilityParams, "status/mobility_params", 1)

        self.create_subscription(Twist, "cmd_vel", self.cmd_velocity_callback, 1, callback_group=self.group)
        self.create_subscription(Pose, "body_pose", self.body_pose_callback, 1, callback_group=self.group)
        self.create_service(
            Trigger,
            "claim",
            lambda request, response: self.service_wrapper("claim", self.handle_claim, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "release",
            lambda request, response: self.service_wrapper("release", self.handle_release, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "stop",
            lambda request, response: self.service_wrapper("stop", self.handle_stop, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "self_right",
            lambda request, response: self.service_wrapper("self_right", self.handle_self_right, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "sit",
            lambda request, response: self.service_wrapper("sit", self.handle_sit, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "stand",
            lambda request, response: self.service_wrapper("stand", self.handle_stand, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "rollover",
            lambda request, response: self.service_wrapper("rollover", self.handle_rollover, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "power_on",
            lambda request, response: self.service_wrapper("power_on", self.handle_power_on, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "power_off",
            lambda request, response: self.service_wrapper("power_off", self.handle_safe_power_off, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/hard",
            lambda request, response: self.service_wrapper("estop/hard", self.handle_estop_hard, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/gentle",
            lambda request, response: self.service_wrapper("estop/gentle", self.handle_estop_soft, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "estop/release",
            lambda request, response: self.service_wrapper(
                "estop/release", self.handle_estop_disengage, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            Trigger,
            "undock",
            lambda request, response: self.service_wrapper("undock", self.handle_undock, request, response),
            callback_group=self.group,
        )

        self.create_service(
            Trigger,
            "spot_check",
            lambda request, response: self.service_wrapper("spot_check", self.handle_spot_check, request, response),
            callback_group=self.group,
        )

        if has_arm:
            self.create_service(
                Trigger,
                "arm_stow",
                lambda request, response: self.service_wrapper("arm_stow", self.handle_arm_stow, request, response),
                callback_group=self.group,
            )
            self.create_service(
                Trigger,
                "arm_unstow",
                lambda request, response: self.service_wrapper("arm_unstow", self.handle_arm_unstow, request, response),
                callback_group=self.group,
            )
            self.create_service(
                Trigger,
                "arm_carry",
                lambda request, response: self.service_wrapper("arm_carry", self.handle_arm_carry, request, response),
                callback_group=self.group,
            )

            self.create_service(
                Trigger,
                "open_gripper",
                lambda request, response: self.service_wrapper(
                    "open_gripper", self.handle_open_gripper, request, response
                ),
                callback_group=self.group,
            )
            self.create_service(
                Trigger,
                "close_gripper",
                lambda request, response: self.service_wrapper(
                    "close_gripper", self.handle_close_gripper, request, response
                ),
                callback_group=self.group,
            )

        self.create_service(
            SetBool,
            "stair_mode",
            lambda request, response: self.service_wrapper("stair_mode", self.handle_stair_mode, request, response),
            callback_group=self.group,
        )
        self.create_service(
            SetLocomotion,
            "locomotion_mode",
            lambda request, response: self.service_wrapper(
                "locomotion_mode", self.handle_locomotion_mode, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            SetVelocity,
            "max_velocity",
            lambda request, response: self.service_wrapper("max_velocity", self.handle_max_vel, request, response),
            callback_group=self.group,
        )
        self.create_service(
            ClearBehaviorFault,
            "clear_behavior_fault",
            lambda request, response: self.service_wrapper(
                "clear_behavior_fault",
                self.handle_clear_behavior_fault,
                request,
                response,
            ),
            callback_group=self.group,
        )

        self.create_service(
            Trigger,
            "stop_dance",
            lambda request, response: self.service_wrapper("stop_dance", self.handle_stop_dance, request, response),
            callback_group=self.group,
        )
        self.create_service(
            UploadAnimation,
            "upload_animation",
            lambda request, response: self.service_wrapper(
                "upload_animation", self.handle_upload_animation, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            UploadSequence,
            "upload_sequence",
            lambda request, response: self.service_wrapper(
                "upload_sequence", self.handle_upload_sequence, request, response
            ),
            callback_group=self.group,
        )

        self.create_service(
            ListAllDances,
            "list_all_dances",
            lambda request, response: self.service_wrapper(
                "list_all_dances", self.handle_list_all_dances, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListAllMoves,
            "list_all_moves",
            lambda request, response: self.service_wrapper(
                "list_all_moves", self.handle_list_all_moves, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ChoreographyRecordedStateToAnimation,
            "recorded_state_to_animation",
            lambda request, response: self.service_wrapper(
                "recorded_state_to_animation", self.handle_recorded_state_to_animation, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ChoreographyStartRecordingState,
            "start_recording_state",
            lambda request, response: self.service_wrapper(
                "start_recording_state", self.handle_start_recording_state, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ChoreographyStopRecordingState,
            "stop_recording_state",
            lambda request, response: self.service_wrapper(
                "stop_recording_state", self.handle_stop_recording_state, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            GetChoreographyStatus,
            "get_choreography_status",
            lambda request, response: self.service_wrapper(
                "get_choreography_status", self.handle_get_choreography_status, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListSounds,
            "list_sounds",
            lambda request, response: self.service_wrapper("list_sounds", self.handle_list_sounds, request, response),
            callback_group=self.group,
        )
        self.create_service(
            LoadSound,
            "load_sound",
            lambda request, response: self.service_wrapper("load_sound", self.handle_load_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            PlaySound,
            "play_sound",
            lambda request, response: self.service_wrapper("play_sound", self.handle_play_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            DeleteSound,
            "delete_sound",
            lambda request, response: self.service_wrapper("delete_sound", self.handle_delete_sound, request, response),
            callback_group=self.group,
        )
        self.create_service(
            GetVolume,
            "get_volume",
            lambda request, response: self.service_wrapper("get_volume", self.handle_get_volume, request, response),
            callback_group=self.group,
        )
        self.create_service(
            SetVolume,
            "set_volume",
            lambda request, response: self.service_wrapper("set_volume", self.handle_set_volume, request, response),
            callback_group=self.group,
        )
        self.create_service(
            ListPtz,
            "list_ptz",
            lambda request, response: self.service_wrapper("list_ptz", self.handle_list_ptz, request, response),
            callback_group=self.group,
        )
        self.create_service(
            GetPtzPosition,
            "get_ptz_position",
            lambda request, response: self.service_wrapper(
                "get_ptz_position", self.handle_get_ptz_position, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            SetPtzPosition,
            "set_ptz_position",
            lambda request, response: self.service_wrapper(
                "set_ptz_position", self.handle_set_ptz_position, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            InitializeLens,
            "initialize_lens",
            lambda request, response: self.service_wrapper(
                "initialize_lens", self.handle_initialize_lens, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListCameras,
            "list_cameras",
            lambda request, response: self.service_wrapper("list_cameras", self.handle_list_cameras, request, response),
            callback_group=self.group,
        )
        self.create_service(
            ListLogpoints,
            "list_logpoints",
            lambda request, response: self.service_wrapper(
                "list_logpoints", self.handle_list_logpoints, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            RetrieveLogpoint,
            "retrieve_logpoint",
            lambda request, response: self.service_wrapper(
                "retrieve_logpoint", self.handle_retrieve_logpoint, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            GetLogpointStatus,
            "get_logpoint_status",
            lambda request, response: self.service_wrapper(
                "get_logpoint_status", self.handle_get_logpoint_status, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            DeleteLogpoint,
            "delete_logpoint",
            lambda request, response: self.service_wrapper(
                "get_logpoint_status", self.handle_delete_logpoint, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            StoreLogpoint,
            "store_logpoint",
            lambda request, response: self.service_wrapper(
                "store_logpoint", self.handle_store_logpoint, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            TagLogpoint,
            "tag_logpoint",
            lambda request, response: self.service_wrapper("tag_logpoint", self.handle_tag_logpoint, request, response),
            callback_group=self.group,
        )
        self.create_service(
            GetLEDBrightness,
            "get_led_brightness",
            lambda request, response: self.service_wrapper(
                "get_led_brightness", self.handle_get_led_brightness, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            SetLEDBrightness,
            "set_led_brightness",
            lambda request, response: self.service_wrapper(
                "set_led_brightness", self.handle_set_led_brightness, request, response
            ),
            callback_group=self.group,
        )
        self.create_service(
            ListGraph,
            "list_graph",
            lambda request, response: self.service_wrapper("list_graph", self.handle_list_graph, request, response),
            callback_group=self.group,
        )
        self.create_service(
            Dock,
            "dock",
            lambda request, response: self.service_wrapper("dock", self.handle_dock, request, response),
            callback_group=self.group,
        )

        # This doesn't use the service wrapper because it's not a trigger, and we want different mock responses
        self.create_service(ListWorldObjects, "list_world_objects", self.handle_list_world_objects)

        self.create_service(
            GraphNavUploadGraph,
            "graph_nav_upload_graph",
            self.handle_graph_nav_upload_graph,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavClearGraph,
            "graph_nav_clear_graph",
            self.handle_graph_nav_clear_graph,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavGetLocalizationPose,
            "graph_nav_get_localization_pose",
            self.handle_graph_nav_get_localization_pose,
            callback_group=self.group,
        )

        self.create_service(
            GraphNavSetLocalization,
            "graph_nav_set_localization",
            self.handle_graph_nav_set_localization,
            callback_group=self.group,
        )
        if has_arm:
            self.create_service(
                GetGripperCameraParameters,
                "get_gripper_camera_parameters",
                lambda request, response: self.service_wrapper(
                    "get_gripper_camera_parameters",
                    self.handle_get_gripper_camera_parameters,
                    request,
                    response,
                ),
                callback_group=self.group,
            )

            self.create_service(
                SetGripperCameraParameters,
                "set_gripper_camera_parameters",
                lambda request, response: self.service_wrapper(
                    "set_gripper_camera_parameters",
                    self.handle_set_gripper_camera_parameters,
                    request,
                    response,
                ),
                callback_group=self.group,
            )

            self.create_service(
                OverrideGraspOrCarry,
                "override_grasp_or_carry",
                lambda request, response: self.service_wrapper(
                    "override_grasp_or_carry",
                    self.handle_override_grasp_or_carry,
                    request,
                    response,
                ),
                callback_group=self.group,
            )

        self.execute_dance_as = ActionServer(
            self,
            ExecuteDance,
            "execute_dance",
            self.handle_execute_dance,
        )

        self.navigate_as = ActionServer(
            self,
            NavigateTo,
            "navigate_to",
            self.handle_navigate_to,
        )
        # spot_ros.navigate_as.start() # As is online

        self.trajectory_server = ActionServer(
            self,
            Trajectory,
            "trajectory",
            self.handle_trajectory,
        )
        # spot_ros.trajectory_server.start()

        self.create_service(
            RobotCommandService,
            "robot_command",
            lambda request, response: self.service_wrapper(
                "robot_command",
                self.handle_robot_command_service,
                request,
                response,
            ),
            callback_group=self.group,
        )

        if has_arm:
            # Allows both the "robot command" and the "manipulation" action goal to preempt each other
            self.robot_command_and_manipulation_servers = SingleGoalMultipleActionServers(
                self,
                [
                    (
                        RobotCommandAction,
                        "robot_command",
                        self.handle_robot_command_action,
                        None,
                    ),
                    (
                        Manipulation,
                        "manipulation",
                        self.handle_manipulation_command,
                        None,
                    ),
                ],
            )
        else:
            self.robot_command_server = SingleGoalActionServer(
                self,
                RobotCommandAction,
                "robot_command",
                self.handle_robot_command_action,
            )

        # Register Shutdown Handle
        # rclpy.on_shutdown(spot_ros.shutdown) # Shutdown Handle

        # Wait for an estop to be connected
        if self.spot_wrapper is not None and not self.start_estop.value:
            printed = False
            while self.spot_wrapper.is_estopped():
                if not printed:
                    self.get_logger().warn(
                        COLOR_YELLOW
                        + "Waiting for estop to be released.  Make sure you have an active estop."
                        '  You can acquire an estop on the tablet by choosing "Acquire Cut Motor Power Authority"'
                        " in the dropdown menu from the power icon.  (This will not power the motors or take the"
                        " lease.)"
                        + COLOR_END,
                    )
                    printed = True
                time.sleep(0.5)
            self.get_logger().info("Found estop!")

        self.create_timer(1 / self.async_tasks_rate, self.step)

        if self.spot_wrapper is not None and self.auto_claim.value:
            self.spot_wrapper.claim()
            if self.auto_power_on.value:
                self.spot_wrapper.power_on()
                if self.auto_stand.value:
                    self.spot_wrapper.stand()

        self.create_service(
            srv_type=Trigger,
            srv_name="take_lease",
            callback=self.take_lease_callback,
            callback_group=self.group,
        )

    def take_lease_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Incoming request to take a new lease.")
        if self.spot_wrapper is None:
            response.success = True
            response.message = "spot_ros2 is running in mock mode."
            return response

        have_new_lease, lease = self.spot_wrapper.takeLease()
        if have_new_lease:
            response.success = True
            response.message = str(lease.lease_proto)
        else:
            response.success = False
            response.message = ""

        return response

    def metrics_callback(self, results: Any) -> None:
        """Callback for when the Spot Wrapper gets new metrics data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        if self.spot_wrapper is None:
            return

        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)
            metrics_msg.header.stamp = builtin_interfaces.msg.Time(sec=local_time.seconds, nanosec=local_time.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    # metrics_msg.time_moving = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = builtin_interfaces.msg.Duration(
                        sec=metric.duration.seconds, nanosec=metric.duration.nanos
                    )
                    metrics_msg.time_moving = duration
                if metric.label == "electric power":
                    # metrics_msg.electric_power = Time(metric.duration.seconds, metric.duration.nanos)
                    duration = builtin_interfaces.msg.Duration(
                        sec=metric.duration.seconds, nanosec=metric.duration.nanos
                    )
                    metrics_msg.electric_power = duration
            self.metrics_pub.publish(metrics_msg)

    def lease_callback(self, results: Any) -> None:
        """Callback for when the Spot Wrapper gets new lease data.
        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        if self.spot_wrapper is None:
            return

        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
            for resource in lease_list:
                new_resource = LeaseResource()
                new_resource.resource = resource.resource
                new_resource.lease.resource = resource.lease.resource
                new_resource.lease.epoch = resource.lease.epoch

                for seq in resource.lease.sequence:
                    new_resource.lease.sequence.append(seq)

                new_resource.lease_owner.client_name = resource.lease_owner.client_name
                new_resource.lease_owner.user_name = resource.lease_owner.user_name

                lease_array_msg.resources.append(new_resource)

            self.lease_pub.publish(lease_array_msg)

    def publish_graph_nav_pose_callback(self) -> None:
        if self.spot_wrapper is None:
            return

        try:
            # noinspection PyProtectedMember
            state = self.spot_wrapper.spot_graph_nav._graph_nav_client.get_localization_state()
            if not state.localization.waypoint_id:
                self.get_logger().warning("Robot is not localized; Please upload graph and localize.")
                return

            (
                seed_t_body_msg,
                seed_t_body_trans_msg,
            ) = bosdyn_localization_to_pose_msg(
                state.localization,
                self.spot_wrapper.robotToLocalTime,
                in_seed_frame=True,
                seed_frame=self.graph_nav_seed_frame,
                body_frame=self.tf_name_graph_nav_body,
                return_tf=True,
            )
            self.graph_nav_pose_pub.publish(seed_t_body_msg)
            self.graph_nav_pose_transform_broadcaster.sendTransform(seed_t_body_trans_msg)
        except Exception as e:
            self.get_logger().error(f"Exception: {e} \n {traceback.format_exc()}")

    def service_wrapper(
        self,
        name: str,
        handler: Callable[[Request, Response], Response],
        request: Request,
        response: Response,
    ) -> Response:
        if self.spot_wrapper is None:
            self.get_logger().info(f"Mock mode: service {name} successfully called with request {request}")
            response.success = True
            return response
        return handler(request, response)

    def handle_claim(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the claim service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.claim()
        return response

    def handle_release(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the release service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.release()
        return response

    def handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stop service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.stop()
        return response

    def handle_self_right(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the self-right service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.self_right()
        return response

    def handle_sit(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the sit service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.sit()
        return response

    def handle_stand(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stand service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.stand()
        return response

    def handle_rollover(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the rollover service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.battery_change_pose()
        return response

    def handle_power_on(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the power-on service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.power_on()
        return response

    def handle_safe_power_off(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the safe-power-off service"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.safe_power_off()
        return response

    def handle_estop_hard(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.assertEStop(True)
        return response

    def handle_estop_soft(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.assertEStop(False)
        return response

    def handle_estop_disengage(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to disengage the eStop on the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.disengageEStop()
        return response

    def handle_undock(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to undock the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_docking.undock()
        return response

    def handle_spot_check(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to run spot check on the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_check.start_check()
        return response

    def handle_arm_stow(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to stow the arm on the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.arm_stow()
        return response

    def handle_arm_unstow(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to ready (unstow) the arm on the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.arm_unstow()
        return response

    def handle_arm_carry(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to carry an object the robot has already grasped."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.arm_carry()
        return response

    def handle_open_gripper(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to open the gripper."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.gripper_open()
        return response

    def handle_close_gripper(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to close the gripper."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.gripper_close()
        return response

    def handle_clear_behavior_fault(
        self, request: ClearBehaviorFault.Request, response: ClearBehaviorFault.Response
    ) -> ClearBehaviorFault.Response:
        """ROS service handler for clearing behavior faults"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        success, message, cleared = self.spot_wrapper.clear_behavior_fault(request.id)
        if not cleared:
            success = False
            message = "No behavior fault cleared"
        response.success = success
        response.message = message
        return response

    def handle_stop_dance(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """ROS service handler to stop the robot's dancing."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        success, msg = self.spot_wrapper.stop_choreography()
        response.success = success
        response.message = msg
        return response

    def handle_list_all_dances(
        self, request: ListAllDances.Request, response: ListAllDances.Response
    ) -> ListAllDances.Response:
        """ROS service handler for getting list of already uploaded dances."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        (
            response.success,
            response.message,
            response.dances,
        ) = self.spot_wrapper.list_all_dances()
        return response

    def handle_list_all_moves(
        self, request: ListAllMoves.Request, response: ListAllMoves.Response
    ) -> ListAllMoves.Response:
        """ROS service handler for getting list of already uploaded moves."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        (
            response.success,
            response.message,
            response.moves,
        ) = self.spot_wrapper.list_all_moves()
        return response

    def handle_recorded_state_to_animation(
        self,
        request: ChoreographyRecordedStateToAnimation.Request,
        response: ChoreographyRecordedStateToAnimation.Response,
    ) -> ChoreographyRecordedStateToAnimation.Response:
        """ROS service handler for transforming a recorded state log into an animation cha file."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response

        with tempfile.TemporaryDirectory() as temp_dir:
            filename = "temporary_animation"
            full_path = os.path.join(temp_dir, filename)
            response.success, response.message, file_name = self.spot_wrapper.choreography_log_to_animation_file(
                filename, temp_dir, request.has_arm
            )
            if response.success:
                with open(full_path + ".cha", "r") as animation_file:
                    response.animation_file_contents = animation_file.read()
        return response

    def handle_start_recording_state(
        self, request: ChoreographyStartRecordingState.Request, response: ChoreographyStartRecordingState.Response
    ) -> ChoreographyStartRecordingState.Response:
        """ROS service handler to start recording a state log for later animation."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message, start_recording_response = self.spot_wrapper.start_recording_state(
            request.duration_seconds
        )
        response.status = start_recording_response.status
        response.recording_session_id = start_recording_response.recording_session_id
        return response

    def handle_stop_recording_state(
        self, request: ChoreographyStopRecordingState.Request, response: ChoreographyStopRecordingState.Response
    ) -> ChoreographyStopRecordingState.Response:
        """ROS service handler to stop recording state for later animation."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message, _ = self.spot_wrapper.stop_recording_state()
        return response

    def handle_get_choreography_status(
        self, request: GetChoreographyStatus.Request, response: GetChoreographyStatus.Response
    ) -> GetChoreographyStatus.Response:
        """ROS service handler for getting current status of choreography playback."""

        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response

        response.success, response.message, choreography_status = self.spot_wrapper.get_choreography_status()
        response.status = choreography_status.status
        response.execution_id = choreography_status.execution_id
        return response

    def handle_upload_animation(
        self, request: UploadAnimation.Request, response: UploadAnimation.Response
    ) -> UploadAnimation.Response:
        """ROS service handler for uploading an animation."""

        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        if request.animation_file_content:
            response.success, response.message = self.spot_wrapper.upload_animation(
                request.animation_name, request.animation_file_content
            )
        elif request.animation_proto_serialized:
            animation = Animation()
            animation.ParseFromString(bytes(bytearray(request.animation_proto_serialized)))
            response.success, response.message = self.spot_wrapper.upload_animation_proto(animation)
        else:
            self.message = "Error: No data passed in message"
            response.success = False
        return response

    def handle_upload_sequence(
        self, request: UploadSequence.Request, response: UploadSequence.Response
    ) -> UploadSequence.Response:
        """ROS service handler for uploading choreography."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        elif request.sequence_proto_serialized:
            sequence = ChoreographySequence()
            sequence.ParseFromString(bytes(bytearray(request.sequence_proto_serialized)))
            response.success, response.message = self.spot_wrapper.upload_choreography(sequence)
        else:
            self.message = "Error: No data passed in message"
            response.success = False
        return response

    def handle_list_sounds(self, request: ListSounds.Request, response: ListSounds.Response) -> ListSounds.Response:
        """ROS service handler for listing sounds loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            names = self.spot_cam_wrapper.audio.list_sounds()
            response.names = names
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_load_sound(self, request: LoadSound.Request, response: LoadSound.Response) -> LoadSound.Response:
        """ROS service handler for loading a wav file sound on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.load_sound(request.wav_path, request.name)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_play_sound(self, request: PlaySound.Request, response: PlaySound.Response) -> PlaySound.Response:
        """ROS service handler for playing a sound loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.play_sound(request.name, request.volume_multiplier)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_delete_sound(self, request: DeleteSound.Request, response: DeleteSound.Response) -> DeleteSound.Response:
        """ROS service handler for deleting a sound loaded on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.delete_sound(request.name)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_get_volume(self, request: GetVolume.Request, response: GetVolume.Response) -> GetVolume.Response:
        """ROS service handler for getting the volume on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            response.volume = self.spot_cam_wrapper.audio.get_volume()
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_set_volume(self, request: SetVolume.Request, response: SetVolume.Response) -> SetVolume.Response:
        """ROS service handler for setting the volume on Spot CAM."""
        if self.spot_cam_wrapper is None:
            response.success = False
            response.message = "Spot CAM has not been initialized"
            return response

        try:
            self.spot_cam_wrapper.audio.set_volume(request.volume)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_list_ptz(self, request: ListPtz.Request, response: ListPtz.Response) -> ListPtz.Response:
        """Ros service handler for getting descriptions of any ptz"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_descriptions = self.spot_cam_wrapper.ptz.list_ptz()
            descriptions = []
            for proto_description in proto_descriptions:
                ros_msg = PtzDescription()
                convert(proto_description, ros_msg)
                descriptions.append(ros_msg)
            response.success = True
            response.message = "Success"
            response.descriptions = descriptions
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_get_ptz_position(
        self, request: GetPtzPosition.Request, response: GetPtzPosition.Response
    ) -> GetPtzPosition.Response:
        """Ros service handler to get the position of a ptz camera"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_position = self.spot_cam_wrapper.ptz.get_ptz_position(request.name)
            convert(proto_position, response.position)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_set_ptz_position(
        self, request: SetPtzPosition.Request, response: SetPtzPosition.Response
    ) -> SetPtzPosition.Response:
        """Ros service handler for setting the position of a ptz camera"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            self.spot_cam_wrapper.ptz.set_ptz_position(request.name, request.pan, request.tilt, request.zoom)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_initialize_lens(
        self, request: InitializeLens.Request, response: InitializeLens.Response
    ) -> InitializeLens.Response:
        """Ros service handler for initializing the lens"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            self.spot_cam_wrapper.ptz.initialise_lens()  # British spelling?
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_list_cameras(self, request: ListCameras.Request, response: ListCameras.Response) -> ListCameras.Response:
        """Ros service handler for listing all cameras on SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_cameras = self.spot_cam_wrapper.media_log.list_cameras()
            cameras = []
            for proto_camera in proto_cameras:
                ros_msg = Camera()
                convert(proto_camera, ros_msg)
                cameras.append(ros_msg)
            response.success = True
            response.message = "Success"
            response.cameras = cameras
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_list_logpoints(
        self, request: ListLogpoints.Request, response: ListLogpoints.Response
    ) -> ListLogpoints.Response:
        """Ros service handler for listing all logpoints saved on SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_logpoints = self.spot_cam_wrapper.media_log.list_logpoints()
            logpoints = []
            for proto_logpoint in proto_logpoints:
                ros_msg = Logpoint()
                convert(proto_logpoint, ros_msg)
                logpoints.append(ros_msg)
            response.success = True
            response.message = "Success"
            response.logpoints = logpoints
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_retrieve_logpoint(
        self, request: RetrieveLogpoint.Request, response: RetrieveLogpoint.Response
    ) -> RetrieveLogpoint.Response:
        """Ros service handler for retrieving a logpoint from SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_logpoint, proto_data_chunk = self.spot_cam_wrapper.media_log.retrieve_logpoint(
                request.name, request.raw
            )
            convert(proto_logpoint, response.logpoint)
            # Data is actually a bytes object, not DataChunk as the SpotCAM wrapper states...
            # Therefore, we use a uint8[] buffer in srv message and directly set that
            # to the bytes object.
            response.data = proto_data_chunk
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_get_logpoint_status(
        self, request: GetLogpointStatus.Request, response: GetLogpointStatus.Response
    ) -> GetLogpointStatus.Response:
        """Ros service handler for getting the status of a logpoint from SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_logstatus = self.spot_cam_wrapper.media_log.get_logpoint_status(request.name)
            response.status.value = proto_logstatus.status  # Manual proto conversion
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_delete_logpoint(
        self, request: DeleteLogpoint.Request, response: DeleteLogpoint.Response
    ) -> DeleteLogpoint.Response:
        """Ros service handler for deleting a logpoint from SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            self.spot_cam_wrapper.media_log.delete_logpoint(request.name)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_store_logpoint(
        self, request: StoreLogpoint.Request, response: StoreLogpoint.Response
    ) -> StoreLogpoint.Response:
        """Ros service handler for storing current camera data as a logpoint on SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            tag = None if request.tag == "" else request.tag
            camera_name = SpotCamCamera(request.name)  # Silly but don't want to modify cam wrapper.
            proto_logpoint = self.spot_cam_wrapper.media_log.store(camera_name, tag)
            convert(proto_logpoint, response.logpoint)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_tag_logpoint(self, request: TagLogpoint.Request, response: TagLogpoint.Response) -> TagLogpoint.Response:
        """Ros service handler for adding a tag to a logpoint on SpotCAM"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            self.spot_cam_wrapper.media_log.tag(request.name, request.tag)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_get_led_brightness(
        self, request: GetLEDBrightness.Request, response: GetLEDBrightness.Response
    ) -> GetLEDBrightness.Response:
        """Ros service handler for getting the current brightness of the Spot CAM onboard LEDs"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            proto_brightness_list = self.spot_cam_wrapper.lighting.get_led_brightness()
            response.success = True
            response.message = "Success"
            response.brightness = proto_brightness_list
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_set_led_brightness(
        self, request: SetLEDBrightness.Request, response: SetLEDBrightness.Response
    ) -> SetLEDBrightness.Response:
        """Ros service handler to set the brightness of Spot CAM's onboard LEDS"""
        try:
            if self.spot_cam_wrapper is None:
                raise Exception("Spot CAM has not been initialized")

            self.spot_cam_wrapper.lighting.set_led_brightness(request.brightness)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def handle_stair_mode(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """ROS service handler to set a stair mode to the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = request.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = "Error:{}".format(e)
            return response

    def handle_locomotion_mode(
        self, request: SetLocomotion.Request, response: SetLocomotion.Response
    ) -> SetLocomotion.Response:
        """ROS service handler to set locomotion mode"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = request.locomotion_mode
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = "Error:{}".format(e)
            return response

    def handle_dock(self, request: Dock.Request, response: Dock.Response) -> Dock.Response:
        """ROS service handler to dock the robot."""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        response.success, response.message = self.spot_wrapper.spot_docking.dock(request.dock_id)
        return response

    def handle_max_vel(self, request: SetVelocity.Request, response: SetVelocity.Response) -> SetVelocity.Response:
        """
        Handle a max_velocity service call. This will modify the mobility params to have a limit on the maximum
        velocity that the robot can move during motion commands. This affects trajectory commands and velocity
        commands
        Args:
            response: SetVelocity.Response containing response
            request: SetVelocity.Request containing requested maximum velocity
        Returns: SetVelocity.Response
        """
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is undefined"
            return response
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(
                    max_vel=math_helpers.SE2Velocity(
                        request.velocity_limit.linear.x,
                        request.velocity_limit.linear.y,
                        request.velocity_limit.angular.z,
                    ).to_proto()
                )
            )
            self.spot_wrapper.set_mobility_params(mobility_params)
            response.success = True
            response.message = "Success"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Error: {e}"
            return response

    def _process_feedback_status(self, status: int) -> Optional[GoalResponse]:
        if status == RobotCommandFeedbackStatusStatus.STATUS_UNKNOWN:
            return GoalResponse.IN_PROGRESS

        if status == RobotCommandFeedbackStatusStatus.STATUS_COMMAND_OVERRIDDEN:
            self.get_logger().warn("Command has been overwritten")
            return GoalResponse.FAILED

        if status == RobotCommandFeedbackStatusStatus.STATUS_COMMAND_TIMED_OUT:
            self.get_logger().warn("Command has timed out")
            return GoalResponse.FAILED

        if status == RobotCommandFeedbackStatusStatus.STATUS_ROBOT_FROZEN:
            self.get_logger().warn("Robot is in unsafe state. Will only respond to safe commands")
            return GoalResponse.FAILED

        if status == RobotCommandFeedbackStatusStatus.STATUS_INCOMPATIBLE_HARDWARE:
            self.get_logger().warn("Command is incompatible with current hardware")
            return GoalResponse.FAILED

        # if status == RobotCommandFeedbackStatusStatus.STATUS_PROCESSING,
        # return None to continue processing the command feedback
        return None

    def _process_full_body_command_feedback(self, feedback: FullBodyCommandFeedback) -> GoalResponse:
        maybe_goal_response = self._process_feedback_status(feedback.status.value)
        if maybe_goal_response is not None:
            return maybe_goal_response

        fb = feedback.feedback
        choice = fb.feedback_choice

        if choice == fb.FEEDBACK_STOP_FEEDBACK_SET:
            return GoalResponse.SUCCESS
        elif choice == fb.FEEDBACK_FREEZE_FEEDBACK_SET:
            return GoalResponse.SUCCESS
        elif choice == fb.FEEDBACK_SELFRIGHT_FEEDBACK_SET:
            if fb.selfright_feedback.status.value == fb.selfright_feedback.status.STATUS_COMPLETED:
                return GoalResponse.SUCCESS
            return GoalResponse.IN_PROGRESS

        elif choice == fb.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET:
            if fb.safe_power_off_feedback.status.value == fb.safe_power_off_feedback.status.STATUS_POWERED_OFF:
                return GoalResponse.SUCCESS
            return GoalResponse.IN_PROGRESS

        elif choice == fb.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET:
            if fb.battery_change_pose_feedback.status.value == fb.battery_change_pose_feedback.status.STATUS_COMPLETED:
                return GoalResponse.SUCCESS
            if fb.battery_change_pose_feedback.status.value == fb.battery_change_pose_feedback.status.STATUS_FAILED:
                return GoalResponse.FAILED
            return GoalResponse.IN_PROGRESS

        elif choice == fb.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET:
            if fb.payload_estimation_feedback.status.value == fb.payload_estimation_feedback.status.STATUS_COMPLETED:
                return GoalResponse.SUCCESS
            if fb.payload_estimation_feedback.status.value == fb.payload_estimation_feedback.status.STATUS_SMALL_MASS:
                return GoalResponse.SUCCESS
            if fb.payload_estimation_feedback.status.value == fb.payload_estimation_feedback.status.STATUS_ERROR:
                return GoalResponse.FAILED
            return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET:
            if (
                fb.constrained_manipulation_feedback.status.value
                == fb.constrained_manipulation_feedback.status.STATUS_RUNNING
            ):
                return GoalResponse.IN_PROGRESS
            return GoalResponse.FAILED
        return GoalResponse.IN_PROGRESS

    def _process_synchronized_arm_command_feedback(self, feedback: ArmCommandFeedback) -> GoalResponse:
        maybe_goal_response = self._process_feedback_status(feedback.status.value)
        if maybe_goal_response is not None:
            return maybe_goal_response

        fb = feedback.feedback
        choice = fb.feedback_choice

        if choice == fb.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET:
            if (
                fb.arm_cartesian_feedback.status.value == fb.arm_cartesian_feedback.status.STATUS_TRAJECTORY_CANCELLED
                or fb.arm_cartesian_feedback.status.value == fb.arm_cartesian_feedback.status.STATUS_TRAJECTORY_STALLED
            ):
                return GoalResponse.FAILED
            if fb.arm_cartesian_feedback.status.value != fb.arm_cartesian_feedback.status.STATUS_TRAJECTORY_COMPLETE:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET:
            if fb.arm_joint_move_feedback.status.value == fb.arm_joint_move_feedback.status.STATUS_STALLED:
                return GoalResponse.FAILED
            if fb.arm_joint_move_feedback.status.value != fb.arm_joint_move_feedback.status.STATUS_COMPLETE:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET:
            if (
                fb.named_arm_position_feedback.status.value
                == fb.named_arm_position_feedback.status.STATUS_STALLED_HOLDING_ITEM
            ):
                return GoalResponse.FAILED
            if fb.named_arm_position_feedback.status.value != fb.named_arm_position_feedback.status.STATUS_COMPLETE:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET:
            self.get_logger().warn("WARNING: ArmVelocityCommand provides no feedback")
        elif choice == fb.FEEDBACK_ARM_GAZE_FEEDBACK_SET:
            if fb.arm_gaze_feedback.status.value == fb.arm_gaze_feedback.status.STATUS_TOOL_TRAJECTORY_STALLED:
                return GoalResponse.FAILED
            if fb.arm_gaze_feedback.status.value != fb.arm_gaze_feedback.status.STATUS_TRAJECTORY_COMPLETE:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_ARM_STOP_FEEDBACK_SET:
            self.get_logger().warn("WARNING: Stop command provides no feedback")
        elif choice == fb.FEEDBACK_ARM_DRAG_FEEDBACK_SET:
            if fb.arm_drag_feedback.status.value == fb.arm_drag_feedback.status.STATUS_DRAGGING:
                return GoalResponse.IN_PROGRESS
            else:
                return GoalResponse.FAILED
        elif choice == fb.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET:
            if (
                fb.arm_impedance_feedback.status.value == fb.arm_impedance_feedback.status.STATUS_TRAJECTORY_CANCELLED
                or fb.arm_impedance_feedback.status.value == fb.arm_impedance_feedback.status.STATUS_TRAJECTORY_STALLED
                or fb.arm_impedance_feedback.status.value == fb.arm_impedance_feedback.status.STATUS_UNKNOWN
            ):
                return GoalResponse.FAILED
            if fb.arm_impedance_feedback.status.value != fb.arm_impedance_feedback.status.STATUS_TRAJECTORY_COMPLETE:
                return GoalResponse.IN_PROGRESS
        else:
            self.get_logger().error("ERROR: unknown arm command type")
            return GoalResponse.IN_PROGRESS
        return GoalResponse.SUCCESS

    def _process_synchronized_mobility_command_feedback(self, feedback: MobilityCommandFeedback) -> GoalResponse:
        maybe_goal_response = self._process_feedback_status(feedback.status.value)
        if maybe_goal_response is not None:
            return maybe_goal_response

        fb = feedback.feedback
        choice = fb.feedback_choice

        if choice == fb.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET:
            if fb.se2_trajectory_feedback.status.value != fb.se2_trajectory_feedback.status.STATUS_AT_GOAL:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET:
            self.get_logger().warn("WARNING: Planar velocity commands provide no feedback")
        elif choice == fb.FEEDBACK_SIT_FEEDBACK_SET:
            if fb.sit_feedback.status.value != fb.sit_feedback.status.STATUS_IS_SITTING:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_STAND_FEEDBACK_SET:
            if fb.stand_feedback.status.value != fb.stand_feedback.status.STATUS_IS_STANDING:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_STANCE_FEEDBACK_SET:
            if fb.stance_feedback.status.value == fb.stance_feedback.status.STATUS_TOO_FAR_AWAY:
                return GoalResponse.FAILED
            if fb.stance_feedback.status.value != fb.stance_feedback.status.STATUS_STANCED:
                return GoalResponse.IN_PROGRESS
        elif choice == fb.FEEDBACK_STOP_FEEDBACK_SET:
            self.get_logger().warn("WARNING: Stop command provides no feedback")
        elif choice == fb.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET:
            self.get_logger().warn("WARNING: FollowArmCommand provides no feedback")
        elif choice == fb.FEEDBACK_NOT_SET:
            # sync_feedback.mobility_command_feedback is set, but feedback_choice is actually not set.
            # This may happen when a command finishes, which means we may return SUCCESS below.
            self.get_logger().info("mobility command feedback indicates goal has reached")
        else:
            self.get_logger().error("ERROR: unknown mobility command type")
            return GoalResponse.IN_PROGRESS
        return GoalResponse.SUCCESS

    def _process_synchronized_gripper_command_feedback(self, feedback: GripperCommandFeedback) -> GoalResponse:
        maybe_goal_response = self._process_feedback_status(feedback.status.value)
        if maybe_goal_response is not None:
            return maybe_goal_response

        if feedback.command.command_choice == feedback.command.COMMAND_CLAW_GRIPPER_FEEDBACK_SET:
            if (
                feedback.command.claw_gripper_feedback.status.value
                == feedback.command.claw_gripper_feedback.status.STATUS_IN_PROGRESS
            ):
                return GoalResponse.IN_PROGRESS
            if (
                feedback.command.claw_gripper_feedback.status.value
                == feedback.command.claw_gripper_feedback.status.STATUS_UNKNOWN
            ):
                self.get_logger().error("ERROR: claw grippper status unknown")
                return GoalResponse.IN_PROGRESS
            if (
                feedback.command.claw_gripper_feedback.status.value
                == feedback.command.claw_gripper_feedback.status.STATUS_AT_GOAL
                or feedback.command.claw_gripper_feedback.status.value
                == feedback.command.claw_gripper_feedback.status.STATUS_APPLYING_FORCE
            ):
                return GoalResponse.SUCCESS
        else:
            self.get_logger().error("ERROR: unknown gripper command type")
            return GoalResponse.IN_PROGRESS
        return GoalResponse.SUCCESS

    def _robot_command_goal_complete(self, feedback: RobotCommandFeedback) -> GoalResponse:
        if feedback is None:
            # NOTE: it takes an iteration for the feedback to get set.
            return GoalResponse.IN_PROGRESS

        choice = feedback.command.command_choice
        if choice == feedback.command.COMMAND_FULL_BODY_FEEDBACK_SET:
            full_body_feedback = feedback.command.full_body_feedback
            return self._process_full_body_command_feedback(full_body_feedback)

        elif choice == feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET:
            # The idea here is that a synchronized command can have arm, mobility, and/or gripper
            # sub-commands in it.  The total command isn't done until all sub-commands are satisfied.
            # So if any one of the sub-commands is still in progress, it short-circuits out as
            # IN_PROGRESS.  And if it makes it to the bottom of the function, then all components
            # must be satisfied, and it returns SUCCESS.
            # One corner case to know about is that the commands that don't provide feedback, such
            # as a velocity command will return SUCCESS.  This allows you to use them more effectively
            # with other commands.  For example if you wanted to move the arm with some velocity
            # while the mobility is going to some SE2 trajectory then that will work.

            sync_feedback = feedback.command.synchronized_feedback
            if sync_feedback.has_field & sync_feedback.ARM_COMMAND_FEEDBACK_FIELD_SET:
                arm_feedback = sync_feedback.arm_command_feedback
                response = self._process_synchronized_arm_command_feedback(arm_feedback)
                if response is not GoalResponse.SUCCESS:
                    return response

            if sync_feedback.has_field & sync_feedback.MOBILITY_COMMAND_FEEDBACK_FIELD_SET:
                mob_feedback = sync_feedback.mobility_command_feedback
                response = self._process_synchronized_mobility_command_feedback(mob_feedback)
                if response is not GoalResponse.SUCCESS:
                    return response

            if sync_feedback.has_field & sync_feedback.GRIPPER_COMMAND_FEEDBACK_FIELD_SET:
                grip_feedback = sync_feedback.gripper_command_feedback
                response = self._process_synchronized_gripper_command_feedback(grip_feedback)
                if response is not GoalResponse.SUCCESS:
                    return response

            return response
        else:
            self.get_logger().error("ERROR: unknown robot command type")
            return GoalResponse.IN_PROGRESS

    def _get_robot_command_feedback(self, goal_id: Optional[str]) -> RobotCommandFeedback:
        feedback = RobotCommandFeedback()
        if goal_id is None:
            mobility_command_feedback = feedback.command.synchronized_feedback.mobility_command_feedback
            feedback.command.command_choice = feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET
            mobility_command_feedback.status.value = mobility_command_feedback.status.STATUS_PROCESSING
            mobility_command_feedback.feedback.feedback_choice = (
                mobility_command_feedback.feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET
            )
            if self._wait_for_goal is not None and self._wait_for_goal.at_goal:
                mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
                    mobility_command_feedback.feedback.se2_trajectory_feedback.status.STATUS_AT_GOAL
                )
            else:
                mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
                    mobility_command_feedback.feedback.se2_trajectory_feedback.status.STATUS_GOING_TO_GOAL
                )
        else:
            if self.spot_wrapper is not None:
                convert(self.spot_wrapper.get_robot_command_feedback(goal_id).feedback, feedback)
        return feedback

    def handle_robot_command_service(
        self, request: RobotCommandService.Request, response: RobotCommandService.Response
    ) -> RobotCommandService.Response:
        proto_command = robot_command_pb2.RobotCommand()
        convert(request.command, proto_command)
        duration = rclpy.duration.Duration.from_msg(request.duration)
        if self.spot_wrapper is not None:
            args = (duration.nanoseconds / 1e9,) if duration.nanoseconds else ()
            response.success, response.message, robot_command_id = self.spot_wrapper.robot_command(proto_command, *args)
            if robot_command_id is not None:
                response.robot_command_id = robot_command_id
        else:
            response.success = True
        return response

    def handle_robot_command_action(self, goal_handle: ServerGoalHandle) -> RobotCommandAction.Result:
        """
        Spot cannot process long trajectories. If we issue a command with long
        trajectories for the arm or the body, the command will be batched,
        assuming that there is only one long trajectory or more but all time
        aligned.
        To account for the network latency, a command must contain batched
        trajectory with some overlapping.
        This is currently not possible for the gripper trajectory, due to some
        limitation in the SDK.
        """
        if self.spot_wrapper is None:
            return

        ros_command = goal_handle.request.command
        proto_command = robot_command_pb2.RobotCommand()
        convert(ros_command, proto_command)

        # Inspect the command and if there are long trajectories, batch them.
        commands = robot_command_util.batch_command(
            proto_command, self.trajectory_batch_size, self.trajectory_batch_overlapping_points
        )
        num_of_commands = len(commands)

        goal_id = None
        self._wait_for_goal = None
        feedback: Optional[RobotCommandFeedback] = None
        feedback_msg: Optional[RobotCommandAction.Feedback] = None

        start_time = time.time()
        time_to_send_command = 0.0

        index = 0
        while (
            rclpy.ok()
            and goal_handle.is_active
            and not goal_handle.is_cancel_requested
            and self._robot_command_goal_complete(feedback) == GoalResponse.IN_PROGRESS
        ):
            # We keep looping and send batches at the expected times until the
            # last batch succeeds. We always send the next batch before the
            # previous one succeeds, so the only batch that can actuallly
            # succeed is the last one.

            time_since_start = time.time() - start_time
            if time_since_start >= time_to_send_command:
                success, err_msg, goal_id = self.spot_wrapper.robot_command(commands[index])
                if not success:
                    raise Exception(err_msg)
                index += 1
                if index < num_of_commands:
                    time_to_send_command = robot_command_util.min_time_since_reference(commands[index])
                else:
                    time_to_send_command = float("inf")
                self.get_logger().info("Robot now executing goal " + str(goal_id))

            feedback = self._get_robot_command_feedback(goal_id)
            feedback_msg = RobotCommandAction.Feedback(feedback=feedback)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.01)  # don't use rate here because we're already in a single thread

        result = RobotCommandAction.Result()
        if feedback is not None:
            goal_handle.publish_feedback(feedback_msg)
            result.result = feedback

        result.success = self._robot_command_goal_complete(feedback) == GoalResponse.SUCCESS

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
            if self.spot_wrapper is not None:
                _, stop_message = self.spot_wrapper.stop()
                self.get_logger().info(f"Stop attempt due to cancellation: {stop_message}")
        elif not goal_handle.is_active:
            result.success = False
            result.message = "Cancelled"
            # Don't abort because that's already happened
        elif result.success:
            result.message = "Successfully completed command"
            goal_handle.succeed()
        else:
            result.message = "Failed to complete command"
            goal_handle.abort()
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self.get_logger().info("Returning action result " + str(result))
        return result

    def _manipulation_goal_complete(self, feedback: Optional[ManipulationApiFeedbackResponse]) -> GoalResponse:
        if feedback is None:
            # NOTE: it takes an iteration for the feedback to get set.
            return GoalResponse.IN_PROGRESS

        if feedback.current_state.value == feedback.current_state.MANIP_STATE_UNKNOWN:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_DONE:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_SEARCHING_FOR_GRASP:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_MOVING_TO_GRASP:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASPING_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACING_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_SUCCEEDED:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_FAILED:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_SUCCEEDED:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_NO_SOLUTION:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_FAILED_TO_RAYCAST_INTO_MAP:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_GRASP_PLANNING_WAITING_DATA_AT_EDGE:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_WALKING_TO_OBJECT:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_ATTEMPTING_RAYCASTING:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_MOVING_TO_PLACE:
            return GoalResponse.IN_PROGRESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_FAILED_TO_RAYCAST_INTO_MAP:
            return GoalResponse.FAILED
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_SUCCEEDED:
            return GoalResponse.SUCCESS
        elif feedback.current_state.value == feedback.current_state.MANIP_STATE_PLACE_FAILED:
            return GoalResponse.FAILED
        else:
            raise Exception("Unknown manipulation state type")

    def _get_manipulation_command_feedback(self, goal_id: str) -> ManipulationApiFeedbackResponse:
        feedback = ManipulationApiFeedbackResponse()
        if self.spot_wrapper is not None:
            convert(self.spot_wrapper.get_manipulation_command_feedback(goal_id), feedback)
        return feedback

    def handle_manipulation_command(self, goal_handle: ServerGoalHandle) -> Manipulation.Result:
        # Most of the logic here copied from handle_robot_command
        self.get_logger().debug("I'm a function that handles request to the manipulation api!")

        ros_command = goal_handle.request.command
        proto_command = manipulation_api_pb2.ManipulationApiRequest()
        convert(ros_command, proto_command)
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self._wait_for_goal = WaitForGoal(self.get_clock(), 2.0)
            goal_id: Optional[str] = None
        else:
            success, err_msg, goal_id = self.spot_wrapper.manipulation_command(proto_command)
            if not success:
                raise Exception(err_msg)

        self.get_logger().info("Robot now executing goal " + str(goal_id))
        # The command is non-blocking, but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        feedback: Optional[ManipulationApiFeedbackResponse] = None
        feedback_msg: Optional[Manipulation.Feedback] = None
        while (
            rclpy.ok()
            and not goal_handle.is_cancel_requested
            and self._manipulation_goal_complete(feedback) == GoalResponse.IN_PROGRESS
            and goal_handle.is_active
        ):
            try:
                if goal_id is not None:
                    feedback = self._get_manipulation_command_feedback(goal_id)
            except InternalServerError as e:
                self.get_logger().error(e)

            feedback_msg = Manipulation.Feedback(feedback=feedback)
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)  # don't use rate here because we're already in a single thread

        # publish a final feedback
        result = Manipulation.Result()
        if feedback is not None:
            goal_handle.publish_feedback(feedback_msg)
            result.result = feedback
        result.success = self._manipulation_goal_complete(feedback) == GoalResponse.SUCCESS

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "Cancelled"
            goal_handle.canceled()
            if self.spot_wrapper is not None:
                _, stop_message = self.spot_wrapper.stop()
                self.get_logger().info(f"Stop attempt due to cancellation: {stop_message}")
        elif not goal_handle.is_active:
            result.success = False
            result.message = "Cancelled"
            # Don't abort because that's already happened
        elif result.success:
            result.message = "Successfully completed manipulation"
            goal_handle.succeed()
        else:
            result.message = "Failed to complete manipulation"
            goal_handle.abort()
        self._wait_for_goal = None
        if not self.spot_wrapper:
            self.get_logger().info("Returning action result " + str(result))
        return result

    def handle_trajectory(self, goal_handle: ServerGoalHandle) -> Optional[Trajectory.Result]:
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        result: Optional[Trajectory.Result] = None

        if goal_handle.request.target_pose.header.frame_id != "body":
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "frame_id of target_pose must be 'body'"
            return result

        if goal_handle.request.duration.sec <= 0:
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "duration must be larger than 0"
            return result

        if self.spot_wrapper is None:
            goal_handle.abort()
            result = Trajectory.Result()
            result.success = False
            result.message = "Spot wrapper is None"
            return result

        cmd_duration_secs = goal_handle.request.duration.sec * 1.0

        self.spot_wrapper.trajectory_cmd(
            goal_x=goal_handle.request.target_pose.pose.position.x,
            goal_y=goal_handle.request.target_pose.pose.position.y,
            goal_heading=math_helpers.Quat(
                w=goal_handle.request.target_pose.pose.orientation.w,
                x=goal_handle.request.target_pose.pose.orientation.x,
                y=goal_handle.request.target_pose.pose.orientation.y,
                z=goal_handle.request.target_pose.pose.orientation.z,
            ).to_yaw(),
            cmd_duration=cmd_duration_secs,
            precise_position=goal_handle.request.precise_positioning,
        )

        command_start_time = self.get_clock().now()

        # Abort the action server if cmd_duration is exceeded - the driver stops but does not provide
        # feedback to indicate this, so we monitor it ourselves
        # The trajectory command is non-blocking, but we need to keep this function up in order to
        # interrupt if a preempt is requested and to return success if/when the robot reaches the goal.
        # Also check the is_active to
        # monitor whether the timeout has already aborted the command

        #
        # Pre-emp missing in port to ROS2 (ros1: self.trajectory_server.is_preempt_requested())
        #

        # rate = rclpy.Rate(10)

        try:
            while rclpy.ok() and not self.spot_wrapper.at_goal and goal_handle.is_active:
                feedback = Trajectory.Feedback()
                if self.spot_wrapper.near_goal:
                    if self.spot_wrapper.last_trajectory_command_precise:
                        feedback.feedback = "Near goal, performing final adjustments"
                    else:
                        feedback.feedback = "Near goal"
                else:
                    feedback.feedback = "Moving to goal"

                # rate.sleep()
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

                # check for timeout
                com_dur = self.get_clock().now() - command_start_time

                if com_dur.nanoseconds / 1e9 > cmd_duration_secs:
                    # timeout, quit with failure
                    self.get_logger().error("TIMEOUT")
                    feedback = Trajectory.Feedback()
                    feedback.feedback = "Failed to reach goal, timed out"
                    goal_handle.publish_feedback(feedback)
                    goal_handle.abort()

            result = Trajectory.Result()
            result.success = False
            result.message = "timeout"

            # If still active after exiting the loop, the command did not time out
            if goal_handle.is_active:
                #            if self.trajectory_server.is_preempt_requested():
                #                self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
                #                self.trajectory_server.set_preempted()
                #                self.spot_wrapper.stop()
                #                result.success = False
                #                result.message = 'preempt'

                feedback = Trajectory.Feedback()
                if self.spot_wrapper.at_goal:
                    # self.get_logger().error("SUCCESS")
                    feedback.feedback = "Reached goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = True
                    result.message = ""
                    goal_handle.succeed()
                else:
                    # self.get_logger().error("FAIL")
                    feedback.feedback = "Failed to reach goal"
                    goal_handle.publish_feedback(feedback)
                    result.success = False
                    result.message = "not at goal"
                    goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f"Exception: {type(e)} - {e}")
            if result is not None:
                result.success = False
            result.message = f"Exception: {type(e)} - {e}"
        # self.get_logger().error(f"RETURN FROM HANDLE: {result}")
        return result

    def cmd_velocity_callback(self, data: Twist) -> None:
        """Callback for cmd_vel command"""
        if not self.spot_wrapper:
            self.get_logger().info(f"Mock mode, received command vel {data}")
            return
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z, self.cmd_duration)

    def body_pose_callback(self, data: Pose) -> None:
        """Callback for cmd_vel command"""
        if self.spot_wrapper is None:
            self.get_logger().info("Mock mode, received command vel " + str(data))
            return
        q = Quaternion()
        q.x = data.orientation.x
        q.y = data.orientation.y
        q.z = data.orientation.z
        q.w = data.orientation.w
        position = geometry_pb2.Vec3(z=data.position.z)
        pose = geometry_pb2.SE3Pose(position=position, rotation=q)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        mobility_params = self.spot_wrapper.get_mobility_params()
        mobility_params.body_control.CopyFrom(body_control)
        self.spot_wrapper.set_mobility_params(mobility_params)

    def handle_graph_nav_get_localization_pose(
        self,
        request: GraphNavGetLocalizationPose.Response,
        response: GraphNavGetLocalizationPose.Response,
    ) -> GraphNavGetLocalizationPose.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if not state.localization.waypoint_id:
                response.success = False
                response.message = "The robot is currently not localized to the map; Please localize."
                self.get_logger().warning(response.message)
                return response
            else:
                seed_t_body_msg = bosdyn_localization_to_pose_msg(
                    state.localization,
                    self.spot_wrapper.robotToLocalTime,
                    in_seed_frame=True,
                    seed_frame=self.graph_nav_seed_frame,
                    body_frame=self.tf_name_graph_nav_body,
                    return_tf=False,
                )
                response.success = True
                response.message = "Success"
                response.pose = seed_t_body_msg
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        if response.success:
            self.get_logger().info("GraphNav localization pose received")
        return response

    def handle_graph_nav_set_localization(
        self,
        request: GraphNavSetLocalization.Request,
        response: GraphNavSetLocalization.Response,
    ) -> GraphNavSetLocalization.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            if request.method == "fiducial":
                self.spot_wrapper.spot_graph_nav.set_initial_localization_fiducial()
                response.success = True
                response.message = "Success"
            elif request.method == "waypoint":
                self.spot_wrapper.spot_graph_nav.set_initial_localization_waypoint([request.waypoint_id])
                response.success = True
                response.message = "Success"
            else:
                response.success = False
                response.message = f"Invalid localization method {request.method}.Must be 'fiducial' or 'waypoint'"
                raise Exception(response.message)
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        if response.success:
            self.get_logger().info(f"Successfully set GraphNav localization. Method: {request.method}")
        return response

    def handle_graph_nav_upload_graph(
        self,
        request: GraphNavUploadGraph.Request,
        response: GraphNavUploadGraph.Response,
    ) -> GraphNavUploadGraph.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().info(f"Uploading GraphNav map: {request.upload_filepath}")
            self.spot_wrapper.spot_graph_nav.upload_graph(request.upload_filepath)
            self.get_logger().info("Uploaded")
            response.success = True
            response.message = "Success"
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        return response

    def handle_graph_nav_clear_graph(
        self, request: GraphNavClearGraph.Request, response: GraphNavClearGraph.Response
    ) -> GraphNavClearGraph.Response:
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().info("Clearing graph")
            self.spot_wrapper.spot_graph_nav.clear_graph()
            self.get_logger().info("Cleared")
            response.success = True
            response.message = "Success"
        except Exception as e:
            self.get_logger().error(f"Exception Error:{e}; \n {traceback.format_exc()}")
            response.success = False
            response.message = f"Exception Error:{e}"
        return response

    def handle_list_graph(self, request: ListGraph.Request, response: ListGraph.Response) -> ListGraph.Response:
        """ROS service handler for listing graph_nav waypoint_ids"""
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response.success = False
            response.message = "Spot wrapper is None"
            return response

        try:
            self.get_logger().info(f"Listing graph for: {request.upload_filepath}")
            self.spot_wrapper.spot_graph_nav.clear_graph()
            self.spot_wrapper.spot_graph_nav.upload_graph(request.upload_filepath)
            response.waypoint_ids = self.spot_wrapper.spot_graph_nav.list_graph()
        except Exception as e:
            self.get_logger().error("Exception Error:{}".format(e))
        return response

    def handle_list_world_objects(
        self, request: ListWorldObjects.Request, response: ListWorldObjects.Response
    ) -> Optional[ListWorldObjects.Response]:
        # For some reason exceptions in service callbacks don't print which makes debugging difficult!
        try:
            return self._handle_list_world_objects(request, response)
        except Exception:
            self.get_logger().error(f"In handling list world objects, exception was {traceback.format_exc()}")
        return None

    def _handle_list_world_objects(
        self, request: ListWorldObjects.Request, response: ListWorldObjects.Response
    ) -> ListWorldObjects.Response:
        object_types = [ot.value for ot in request.request.object_type]
        time_start_point = None
        if request.request.has_field & request.request.TIMESTAMP_FILTER_FIELD_SET:
            time_start_point = (
                request.request.timestamp_filter.sec + float(request.request.timestamp_filter.nanosec) / 1e9
            )
        if self.spot_wrapper is None:
            self.get_logger().info(f"Mock return for {object_types} after time {time_start_point}")
            # return a fake list
            proto_response = world_object_pb2.ListWorldObjectResponse()
            world_object = proto_response.world_objects.add()
            world_object.name = "my_fiducial_3"
            world_object.apriltag_properties.tag_id = 3
            world_object.apriltag_properties.frame_name_fiducial = "fiducial_3"
            world_object.apriltag_properties.frame_name_fiducial_filtered = "filtered_fiducial_3"
        else:
            proto_response = self.spot_wrapper.spot_world_objects.list_world_objects(object_types, time_start_point)
        convert(proto_response, response.response)
        return response

    def handle_execute_dance_feedback(self) -> None:
        """Thread function to send execute dance feedback"""
        if self.spot_wrapper is None:
            return

        while rclpy.ok() and self.run_dance_feedback:
            res, msg, status = self.spot_wrapper.get_choreography_status()
            if res:
                feedback = ExecuteDance.Feedback()
                feedback.is_dancing = status == ChoreographyStatusResponse.Status.STATUS_DANCING
                if self.execute_dance_handle is not None:
                    self.execute_dance_handle.publish_feedback(feedback)

                if status == ChoreographyStatusResponse.Status.STATUS_COMPLETED_SEQUENCE:
                    break

            time.sleep(0.01)

    def handle_execute_dance(self, execute_dance_handle: ServerGoalHandle) -> ExecuteDance.Result:
        """ROS service handler for uploading and executing dance."""

        self.execute_dance_handle = execute_dance_handle

        feedback_thread = threading.Thread(target=self.handle_execute_dance_feedback, args=())
        self.run_dance_feedback = True
        feedback_thread.start()

        if self.spot_wrapper is None:
            error_msg = "Spot wrapper is None"
            self.get_logger().error(error_msg)
            result = ExecuteDance.Result()
            result.success = False
            result.message = error_msg
            return result

        start_slice = 0
        if execute_dance_handle.request.start_slice:
            start_slice = execute_dance_handle.request.start_slice

        # Support different mehtods of starting the dance
        if execute_dance_handle.request.choreo_name:
            res, msg = self.spot_wrapper.execute_choreography_by_name(
                execute_dance_handle.request.choreo_name, start_slice=start_slice
            )

        elif execute_dance_handle.request.choreo_file_content:
            res, msg = self.spot_wrapper.execute_dance(
                execute_dance_handle.request.choreo_file_content, start_slice=start_slice
            )
        elif execute_dance_handle.request.choreo_sequence_serialized:
            sequence = ChoreographySequence()
            sequence.ParseFromString(bytes(bytearray(execute_dance_handle.request.choreo_sequence_serialized)))
            res, msg = self.spot_wrapper.upload_choreography(sequence)
            if res:
                res, msg = self.spot_wrapper.execute_choreography_by_name(sequence.name, start_slice)
        else:
            error_msg = "No dance content sent to execute"
            result = ExecuteDance.Result()
            result.success = False
            result.message = error_msg
            return result

        self.run_dance_feedback = False
        feedback_thread.join()

        execute_dance_handle.succeed()
        result = ExecuteDance.Result()
        result.success = res
        result.message = msg
        return result

    def handle_navigate_to_feedback(self) -> None:
        """Thread function to send navigate_to feedback"""
        if self.spot_wrapper is None:
            return

        while rclpy.ok() and self.run_navigate_to:
            localization_state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if localization_state.localization.waypoint_id:
                feedback = NavigateTo.Feedback()
                feedback.waypoint_id = localization_state.localization.waypoint_id
                if self.goal_handle is not None:
                    self.goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
            # rclpy.Rate(10).sleep()

    def handle_navigate_to(self, goal_handle: ServerGoalHandle) -> NavigateTo.Result:
        """ROS service handler to run mission of the robot.  The robot will replay a mission"""
        # create thread to periodically publish feedback

        self.goal_handle = goal_handle
        feedback_thread = threading.Thread(target=self.handle_navigate_to_feedback, args=())
        self.run_navigate_to = True
        feedback_thread.start()
        if self.spot_wrapper is None:
            self.get_logger().error("Spot wrapper is None")
            response = NavigateTo.Result()
            response.success = False
            response.message = "Spot wrapper is None"
            goal_handle.abort()
            return response

        # run navigate_to
        resp = self.spot_wrapper.spot_graph_nav._navigate_to(
            upload_path=goal_handle.request.upload_path,
            navigate_to=goal_handle.request.navigate_to,
            initial_localization_fiducial=goal_handle.request.initial_localization_fiducial,
            initial_localization_waypoint=goal_handle.request.initial_localization_waypoint,
        )
        self.run_navigate_to = False
        feedback_thread.join()

        result = NavigateTo.Result()
        result.success = resp[0]
        result.message = resp[1]
        # check status
        if resp[0]:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def handle_get_gripper_camera_parameters(
        self,
        request: GetGripperCameraParameters.Request,
        response: GetGripperCameraParameters.Response,
    ) -> GetGripperCameraParameters.Response:
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot Wrapper not initialized"
            return response
        if not self.spot_wrapper.has_arm():
            response.success = False
            response.message = "Spot configuration does not have arm"
            return response
        try:
            proto_request = gripper_camera_param_pb2.GripperCameraGetParamRequest()
            convert(request.request, proto_request)
            proto_response = self.spot_wrapper.spot_images.get_gripper_camera_params(proto_request)
            convert(proto_response, response.response)
            response.success = True
            response.message = "Request to get gripper camera parameters sent"
        except Exception as e:
            error_str = "Error:{}\n{}".format(e, traceback.format_exc())
            self.get_logger().error(error_str)
            response.success = False
            response.message = error_str

        return response

    def handle_set_gripper_camera_parameters(
        self,
        request: SetGripperCameraParameters.Request,
        response: SetGripperCameraParameters.Response,
    ) -> SetGripperCameraParameters.Response:
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot Wrapper not initialized"
            return response
        if not self.spot_wrapper.has_arm():
            response.success = False
            response.message = "Spot configuration does not have arm"
            return response

        try:
            proto_request = gripper_camera_param_pb2.GripperCameraParamRequest()
            convert(request.request, proto_request)
            proto_response = self.spot_wrapper.spot_images.set_gripper_camera_params(proto_request)
            convert(proto_response, response.response)
            response.success = True
            response.message = "Request to set gripper camera parameters sent"
        except Exception as e:
            error_str = "Error:{}\n{}".format(e, traceback.format_exc())
            self.get_logger().error(error_str)
            response.success = False
            response.message = error_str

        return response

    def handle_override_grasp_or_carry(
        self,
        request: OverrideGraspOrCarry.Request,
        response: OverrideGraspOrCarry.Response,
    ) -> OverrideGraspOrCarry.Response:
        response = OverrideGraspOrCarry.Response()
        if self.spot_wrapper is None or not self.spot_wrapper.has_arm():
            response.success = False
            response.message = "Wrapper not available or spot has no arm"
            return response
        response.success, response.message = self.spot_wrapper.spot_arm.override_grasp_or_carry(
            request.grasp_override.value, request.carry_override.value
        )
        return response

    def step(self) -> None:
        """Update spot sensors"""
        if not self._printed_once:
            self.get_logger().info("Driver successfully started!")
            self._printed_once = True
        self.get_logger().debug("Step/Update")
        if rclpy.ok():
            if self.spot_wrapper is not None:
                self.spot_wrapper.updateTasks()  # Testing with Robot
            self.get_logger().debug("UPDATE TASKS")
            feedback_msg = Feedback()
            if self.spot_wrapper:
                feedback_msg.standing = self.spot_wrapper.is_standing
                feedback_msg.sitting = self.spot_wrapper.is_sitting
                feedback_msg.moving = self.spot_wrapper.is_moving
                _id = self.spot_wrapper.id
                try:
                    feedback_msg.serial_number = _id.serial_number
                    feedback_msg.species = _id.species
                    feedback_msg.version = _id.version
                    feedback_msg.nickname = _id.nickname
                    feedback_msg.computer_serial_number = _id.computer_serial_number
                except AttributeError:
                    pass
            self.feedback_pub.publish(feedback_msg)
            mobility_params_msg = MobilityParams()
            if self.spot_wrapper is not None:
                try:
                    mobility_params = self.spot_wrapper.get_mobility_params()
                    mobility_params_msg.body_control.position.x = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.x
                    )
                    mobility_params_msg.body_control.position.y = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.y
                    )
                    mobility_params_msg.body_control.position.z = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.z
                    )
                    mobility_params_msg.body_control.orientation.x = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.x
                    )
                    mobility_params_msg.body_control.orientation.y = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.y
                    )
                    mobility_params_msg.body_control.orientation.z = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.z
                    )
                    mobility_params_msg.body_control.orientation.w = (
                        mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.w
                    )
                    mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint
                    mobility_params_msg.stair_hint = mobility_params.stair_hint
                except Exception as e:
                    self.get_logger().error("Error:{}".format(e))
                    pass
            self.mobility_params_pub.publish(mobility_params_msg)

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down ROS driver for Spot")
        if self.spot_wrapper is not None:
            if self.spot_wrapper.check_is_powered_on() and self.start_estop.value:
                self.get_logger().info("Sitting down...")
                self.spot_wrapper.sit_blocking()
            self.spot_wrapper.disconnect()
        super().destroy_node()


@ros_process.main(prebaked=False)
def main(args: Optional[List[str]] = None) -> None:
    ros_process.spin(SpotROS)


if __name__ == "__main__":
    main()
