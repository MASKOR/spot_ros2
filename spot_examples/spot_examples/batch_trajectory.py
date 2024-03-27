#!/usr/bin/python3

# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

"""
This example shows how long trajectories can be sent to the robot in batches.
The robot draws a curve with the arm and at the same time rotates its body.

Please note that it is not currenlty possible to batch gripper trajectories
due to limitation in the SDK.
"""

import argparse
import math
import time
from typing import Callable, Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import (
    arm_command_pb2,
    basic_command_pb2,
    gripper_command_pb2,
    mobility_command_pb2,
    robot_command_pb2,
    synchronized_command_pb2,
    trajectory_pb2,
)
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
)
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.util import seconds_to_duration, seconds_to_timestamp
from bosdyn_msgs.conversions import convert
from google.protobuf.wrappers_pb2 import DoubleValue
from rclpy.node import Node
from tf2_ros import TransformStamped

from spot_examples.simple_spot_commander import SimpleSpotCommander
from spot_msgs.action import RobotCommand  # type: ignore

ContinuousTrajectory1D = Callable[[float], float]
ContinuousTrajectory2D = Callable[[float], SE2Pose]
ContinuousTrajectory3D = Callable[[float], SE3Pose]


def gripper_continuous_trajectory(t: float) -> float:
    """
    We use this function to model a continuous gripper trajectory over time.
    """
    n = 8
    period = 10.0
    t_norm = t / period
    x = -abs(math.sin(math.pi * n * t_norm))
    return x


def mobility_continuous_trajectory(t: float) -> SE2Pose:
    """
    we use this function to model a continuous mobility trajectory over time.
    """
    period = 10.0
    t_norm = t / period
    x = 0.1
    y = 0.0
    angle = (20 * math.pi / 180) * math.sin(2.0 * math.pi * t_norm)
    return SE2Pose(x, y, angle)


def arm_continuous_trajectory(t: float) -> SE3Pose:
    """
    we use this function to model a continuous arm trajectory over time.
    """

    # Draw a Rhodonea curve with n petals and period P (seconds)
    n = 5
    period = 10.0
    t_norm = t / period
    radius = 0.4 * math.sin(math.pi * n * t_norm)
    x = radius * math.cos(math.pi * t_norm)
    y = radius * math.sin(math.pi * t_norm)
    z = 0.0
    quat = Quat(1, 0, 0, 0)
    return SE3Pose(x, y, z, quat)


def _to_se3(ros_transform: TransformStamped) -> SE3Pose:
    """
    Convert from ROS TransformStamped to Bosdyn SE3Pose
    """
    return SE3Pose(
        ros_transform.transform.translation.x,
        ros_transform.transform.translation.y,
        ros_transform.transform.translation.z,
        Quat(
            ros_transform.transform.rotation.w,
            ros_transform.transform.rotation.x,
            ros_transform.transform.rotation.y,
            ros_transform.transform.rotation.z,
        ),
    )


class SpotRunner:
    """
    This example shows how send a command with very long trajectories.
    """

    def __init__(self, node: Node, args: argparse.Namespace):
        self._node = node
        self._robot_name: str = args.robot
        self._logger = node.get_logger()
        self._robot = SimpleSpotCommander(self._robot_name)

        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )
        self._tf_listener = TFListenerWrapper(self._node)

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._odom_frame_name = namespace_with(self._robot_name, ODOM_FRAME_NAME)
        self._grav_body_frame_name = namespace_with(self._robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)

        self._tf_listener.wait_for_a_tform_b(self._odom_frame_name, self._grav_body_frame_name)
        self._tf_listener.wait_for_a_tform_b(self._vision_frame_name, self._body_frame_name)

    def _ready_arm(self) -> bool:
        """
        Unstow the robot arm.
        """
        command = RobotCommandBuilder.arm_ready_command()
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("ready_arm", action_goal)

    def _arm_stow(self) -> bool:
        """
        Stow the robot arm.
        """
        command = RobotCommandBuilder.arm_stow_command()
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        return self._robot_command_client.send_goal_and_wait("arm_stow", action_goal)

    def _arm_discrete_trajectory(
        self,
        reference_time: float,
        ramp_up_time: float,
        duration: float,
        dt: float,
        trajectory_function: ContinuousTrajectory3D,
    ) -> trajectory_pb2.SE3Trajectory:
        """
        Return a discrete trajectory in 3D space by sampling a continuous 3D
        function. The continuous function is sampled from 0 to a specified duration
        and at given sampling intervals.

        Args:
            reference_time: The time the trajectory is executed.
            ramp_up_time: A delay to let the robot move into the initial position.
            duration: The total sampling time.
            dt: The sampling interval.
            trajectory_function: the trajectory function to sample.
        """
        trajectory = trajectory_pb2.SE3Trajectory()
        trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
        t = 0.0
        while t < duration:
            pos = trajectory_function(t)
            point = trajectory.points.add()
            point.pose.CopyFrom(pos.to_proto())
            point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
            t = t + dt
        return trajectory

    def _gripper_discrete_trajectory(
        self,
        reference_time: float,
        ramp_up_time: float,
        duration: float,
        dt: float,
        trajectory_function: ContinuousTrajectory1D,
    ) -> trajectory_pb2.ScalarTrajectory:
        """
        Return a scalar trajectory by sampling a continuous function. The
        continuous function is sampled from 0 to a specified duration and at given
        sampling intervals.

        Args:
            reference_time: The time the trajectory is executed.
            ramp_up_time: A delay to let the robot move into the initial position.
            duration: The total sampling time.
            dt: The sampling interval.
            trajectory_function: the trajectory function to sample.
        """
        trajectory = trajectory_pb2.ScalarTrajectory()
        trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
        trajectory.interpolation = trajectory_pb2.POS_INTERP_CUBIC
        t = 0.0
        while t < duration:
            pos = trajectory_function(t)
            point = trajectory.points.add()
            point.point = pos
            point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
            t = t + dt
        return trajectory

    def _mobility_discrete_trajectory(
        self,
        reference_time: float,
        ramp_up_time: float,
        duration: float,
        dt: float,
        trajectory_function: ContinuousTrajectory2D,
    ) -> trajectory_pb2.SE2Trajectory:
        """
        Return a discrete trajectory in 2D space by sampling a continuous 2D
        function. The continuous function is sampled from 0 to a specified duration
        and at given sampling intervals.

        Args:
            reference_time: The time the trajectory is executed.
            ramp_up_time: A delay to let the robot move into the initial position.
            duration: The total sampling time.
            dt: The sampling interval.
            trajectory_function: the trajectory function to sample.
        """
        vision_to_body = _to_se3(self._tf_listener.lookup_a_tform_b(self._vision_frame_name, self._body_frame_name))
        trajectory = trajectory_pb2.SE2Trajectory()
        trajectory.reference_time.CopyFrom(seconds_to_timestamp(reference_time))
        t = 0.0
        while t < duration:
            pos = vision_to_body.get_closest_se2_transform() * trajectory_function(t)
            point = trajectory.points.add()
            point.pose.CopyFrom(pos.to_proto())
            point.time_since_reference.CopyFrom(seconds_to_duration(t + ramp_up_time))
            t = t + dt
        return trajectory

    def _build_arm_request(
        self, trajectory: Optional[trajectory_pb2.SE3Trajectory] = None
    ) -> arm_command_pb2.ArmCommand.Request:
        """
        Create a request to make the arm follow the given trajectory.
        """

        # Create a task frame. This will be the frame the trajectory is
        # defined relative to. The center of the frame is 90cm in front of the
        # robot, with z pointing back at the robot, x off the right side of the
        # robot, and y up
        body_to_task = SE3Pose(x=0.9, y=0, z=0, rot=Quat(w=0.5, x=0.5, y=-0.5, z=-0.5))

        # Now, get the transform between the "odometry" frame and the gravity aligned body frame.
        # This will be used in conjunction with the body_to_task frame to get the
        # transformation between the odometry frame and the task frame. In order to get
        # odom_to_body we use a snapshot of the frame tree. For more information on the frame
        # tree, see https://dev.bostondynamics.com/docs/concepts/geometry_and_frames
        odom_to_body: SE3Pose = _to_se3(
            self._tf_listener.lookup_a_tform_b(self._odom_frame_name, self._grav_body_frame_name)
        )

        odom_to_task: SE3Pose = odom_to_body * body_to_task
        wrist_to_tool = SE3Pose(x=0.25, y=0, z=0, rot=Quat(w=0.5, x=0.5, y=-0.5, z=-0.5))

        arm_request = None
        if trajectory is not None:
            arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                root_frame_name=ODOM_FRAME_NAME,
                root_tform_task=odom_to_task.to_proto(),
                wrist_tform_tool=wrist_to_tool.to_proto(),
                pose_trajectory_in_task=trajectory,
                maximum_acceleration=DoubleValue(value=10000.0),
                max_linear_velocity=DoubleValue(value=10000.0),
                max_angular_velocity=DoubleValue(value=10000.0),
            )
            arm_request = arm_command_pb2.ArmCommand.Request(arm_cartesian_command=arm_cartesian_command)
        return arm_request

    def _build_mobility_request(
        self, trajectory: Optional[trajectory_pb2.SE2Trajectory] = None
    ) -> mobility_command_pb2.MobilityCommand.Request:
        """
        Create a request to make the robot follow the given trajectory.
        """
        mobility_request = None
        if trajectory is not None:
            mobility_request = basic_command_pb2.SE2TrajectoryCommand.Request(
                trajectory=trajectory, se2_frame_name=VISION_FRAME_NAME
            )
            mobility_request = mobility_command_pb2.MobilityCommand.Request(se2_trajectory_request=mobility_request)
        return mobility_request

    def _build_gripper_request(
        self, trajectory: Optional[trajectory_pb2.ScalarTrajectory] = None
    ) -> gripper_command_pb2.GripperCommand.Request:
        """
        Create a request to make the gripper follow the given trajectory.
        """
        gripper_request = None
        if trajectory is not None:
            claw_gripper_request = gripper_command_pb2.ClawGripperCommand.Request(trajectory=trajectory)
            gripper_request = gripper_command_pb2.GripperCommand.Request(claw_gripper_command=claw_gripper_request)
        return gripper_request

    def _build_robot_command(
        self,
        hand_trajectory: Optional[trajectory_pb2.SE3Trajectory] = None,
        mobility_trajectory: Optional[trajectory_pb2.SE2Trajectory] = None,
        gripper_trajectory: Optional[trajectory_pb2.ScalarTrajectory] = None,
    ) -> robot_command_pb2.RobotCommand:
        """
        Return a robot command with three optional trajectories to follow.

        Args
            hand_trajectory: A 3D arm trajectory.
            mobility_trajectory: A 2D mobility trajectory.
            gripper_trajectory: A scalar gripper trajectory.

        Returns:
            A robot command with any of the given optional trajectory.
        """

        arm_request = self._build_arm_request(hand_trajectory)
        mobility_request = self._build_mobility_request(mobility_trajectory)
        gripper_request = self._build_gripper_request(gripper_trajectory)

        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_request, mobility_command=mobility_request, gripper_command=gripper_request
        )
        command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
        return command

    def test_run(self) -> bool:
        """
        Send a very long arm trajectory to Spot.
        Returns:
            True the process runs without errors, False otherwise.
        """

        # Claim robot.
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result:
            self._logger.error("Unable to claim robot")
            return False
        self._logger.info("Claimed robot")

        # Power on robot.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result:
            self._logger.error("Unable to power on robot")
            return False

        # Stand up robot.
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result:
            self._logger.error("Robot did not stand")
            return False
        self._logger.info("Successfully stood up.")

        # Unstow the arm.
        self._logger.info("Unstow the arm")
        result = self._ready_arm()
        if not result:
            self._logger.error("Failed to unstow the arm.")
            return False
        self._logger.info("Arm ready.")

        # Make arm, robot and gripper follow the sampled trajectories.

        start_time = time.time()
        self._gripper_discrete_trajectory(
            reference_time=start_time,
            ramp_up_time=4,
            duration=40,
            dt=0.05,
            trajectory_function=gripper_continuous_trajectory,
        )
        mobility_trajectory = self._mobility_discrete_trajectory(
            reference_time=start_time,
            ramp_up_time=4,
            duration=40,
            dt=0.05,
            trajectory_function=mobility_continuous_trajectory,
        )
        hand_trajectory = self._arm_discrete_trajectory(
            reference_time=start_time,
            ramp_up_time=4,
            duration=40,
            dt=0.05,
            trajectory_function=arm_continuous_trajectory,
        )

        # Unfortunately, we cannot send at the moment gripper trajectories with
        # a reference date that falls before the robot time because of a
        # limitation in the SDK.

        command = self._build_robot_command(
            hand_trajectory=hand_trajectory,
            mobility_trajectory=mobility_trajectory,
            # gripper_trajectory=gripper_trajectory,
        )
        action_goal = RobotCommand.Goal()
        convert(command, action_goal.command)
        self._robot_command_client.send_goal_and_wait("move_arm", goal=action_goal)

        # Stow the arm.
        self._logger.info("Stow the arm")
        result = self._arm_stow()
        if not result:
            self._logger.error("Failed to stow the arm.")
            return False
        self._logger.info("Arm stowed.")

        return True


def cli() -> argparse.ArgumentParser:
    """
    Parse all arguments.
    --robot [string]
        The robot name e.g. Opal.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, required=True, help="The robot name.")
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    """
    Execute the example.
    """
    # Set up basic ROS2 utilities for communicating with the driver.
    node = ros_scope.node()
    if node is None:
        raise ValueError("No ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")

    spot_runner = SpotRunner(node, args)
    spot_runner.test_run()


if __name__ == "__main__":
    main(None)
