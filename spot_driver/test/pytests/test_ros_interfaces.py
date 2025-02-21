# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import contextlib
import unittest

import rclpy
import synchros2.scope as ros_scope
from bosdyn_msgs.msg import RobotCommand, RobotCommandFeedback
from std_srvs.srv import Trigger

import spot_driver.spot_ros2
from spot_driver.spot_ros2 import GoalResponse
from spot_msgs.srv import (  # type: ignore
    Dock,
)


class SpotDriverTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = contextlib.ExitStack()
        self.ros = self.fixture.enter_context(ros_scope.top(namespace="fixture"))
        # create and run spot ros2 servers
        self.spot_ros2 = self.ros.load(
            spot_driver.spot_ros2.SpotROS,
            parameter_list=[
                rclpy.parameter.Parameter("spot_name", value="Mock_spot"),
                rclpy.parameter.Parameter("mock_enable", value=True),
                rclpy.parameter.Parameter("mock_has_arm", value=False),
            ],
        )

        # clients
        self.claim_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "claim")
        self.release_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "release")
        self.power_on_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "power_on")
        self.power_off_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "power_off")
        self.sit_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "sit")
        self.stand_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "stand")
        self.estop_gentle: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/gentle")
        self.estop_hard: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/hard")
        self.estop_release: rclpy.node.Client = self.ros.node.create_client(Trigger, "estop/release")
        self.undock_client: rclpy.node.Client = self.ros.node.create_client(Trigger, "undock")
        self.dock_client: rclpy.node.Client = self.ros.node.create_client(Dock, "dock")

    def tearDown(self) -> None:
        self.fixture.close()

    def test_wrapped_commands(self) -> None:
        """
        Spot Ros2 driver has multiple commands that are wrapped in a service wrapper.
        When no spot_wrapper is present they return true, but this test at least tests
        communications and APIs.
        """
        resp = self.claim_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.release_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)

        resp = self.power_on_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.power_off_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.sit_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.stand_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_hard.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_gentle.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.estop_release.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.undock_client.call(Trigger.Request())
        self.assertEqual(resp.success, True)
        resp = self.dock_client.call(Dock.Request())
        self.assertEqual(resp.success, True)

    # Ignore Line too long errors
    # ruff: noqa: E501
    def test_robot_command_goal_complete(self) -> None:
        FEEDBACK_INVALID = -128

        dummy_command = RobotCommand()
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, None), GoalResponse.IN_PROGRESS, "Empty Command"
        )

        feedback = RobotCommandFeedback()

        """ Testing FullBodyFeedback """
        fullbody_feedback = feedback.command.full_body_feedback
        feedback.command.command_choice = feedback.command.COMMAND_FULL_BODY_FEEDBACK_SET

        feedback.command.full_body_feedback.status.value = fullbody_feedback.status.STATUS_UNKNOWN
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "COMMAND_FULL_BODY_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.full_body_feedback.status.value = fullbody_feedback.status.STATUS_PROCESSING

        """ Testing STOP_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_STOP_FEEDBACK_SET",
        )

        """ Testing FREEZE_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_FREEZE_FEEDBACK_SET
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_FREEZE_FEEDBACK_SET",
        )

        """ Testing SELFRIGHT_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_SELFRIGHT_FEEDBACK_SET
        )
        feedback.command.full_body_feedback.feedback.selfright_feedback.status.value = (
            fullbody_feedback.feedback.selfright_feedback.status.STATUS_COMPLETED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_SELFRIGHT_FEEDBACK_SET | STATUS_COMPLETED",
        )

        feedback.command.full_body_feedback.feedback.selfright_feedback.status.value = (
            fullbody_feedback.feedback.selfright_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SELFRIGHT_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.full_body_feedback.feedback.selfright_feedback.status.value = (
            fullbody_feedback.feedback.selfright_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SELFRIGHT_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        """ Testing SAFE_POWER_OFF_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET
        )
        feedback.command.full_body_feedback.feedback.safe_power_off_feedback.status.value = (
            fullbody_feedback.feedback.safe_power_off_feedback.status.STATUS_POWERED_OFF
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET | STATUS_POWERED_OFF",
        )

        feedback.command.full_body_feedback.feedback.safe_power_off_feedback.status.value = (
            fullbody_feedback.feedback.safe_power_off_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.full_body_feedback.feedback.safe_power_off_feedback.status.value = (
            fullbody_feedback.feedback.safe_power_off_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        """ Testing BATTERY_CHANGE_POSE_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET
        )
        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value = (
            fullbody_feedback.feedback.battery_change_pose_feedback.status.STATUS_COMPLETED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET | STATUS_COMPLETED",
        )

        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value = (
            fullbody_feedback.feedback.battery_change_pose_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value = (
            fullbody_feedback.feedback.battery_change_pose_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.full_body_feedback.feedback.battery_change_pose_feedback.status.value = (
            fullbody_feedback.feedback.battery_change_pose_feedback.status.STATUS_FAILED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET | STATUS_FAILED",
        )

        """ Testing PAYLOAD_ESTIMATION_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET
        )
        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value = (
            fullbody_feedback.feedback.payload_estimation_feedback.status.STATUS_COMPLETED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET | STATUS_COMPLETED",
        )

        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value = (
            fullbody_feedback.feedback.payload_estimation_feedback.status.STATUS_SMALL_MASS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET | STATUS_SMALL_MASS",
        )

        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value = (
            fullbody_feedback.feedback.payload_estimation_feedback.status.STATUS_ERROR
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET | STATUS_ERROR",
        )

        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value = (
            fullbody_feedback.feedback.payload_estimation_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.full_body_feedback.feedback.payload_estimation_feedback.status.value = (
            fullbody_feedback.feedback.payload_estimation_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        """ Testing CONSTRAINED_MANIPULATION_FEEDBACK_SET """
        feedback.command.full_body_feedback.feedback.feedback_choice = (
            fullbody_feedback.feedback.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET
        )
        feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.value = (
            fullbody_feedback.feedback.constrained_manipulation_feedback.status.STATUS_RUNNING
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET | STATUS_RUNNING",
        )

        feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.value = (
            fullbody_feedback.feedback.constrained_manipulation_feedback.status.STATUS_GRASP_IS_LOST
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET | STATUS_GRASP_IS_LOST",
        )

        feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.value = (
            fullbody_feedback.feedback.constrained_manipulation_feedback.status.STATUS_ARM_IS_STUCK
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET | STATUS_ARM_IS_STUCK",
        )

        feedback.command.full_body_feedback.feedback.constrained_manipulation_feedback.status.value = (
            fullbody_feedback.feedback.constrained_manipulation_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        """ Testing Synchronized Feedback Command """
        arm_command_feedback = feedback.command.synchronized_feedback.arm_command_feedback
        feedback.command.command_choice = feedback.command.COMMAND_SYNCHRONIZED_FEEDBACK_SET

        """ Testing arm command feedback """
        feedback.command.synchronized_feedback.has_field = (
            feedback.command.synchronized_feedback.ARM_COMMAND_FEEDBACK_FIELD_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.status.value = (
            arm_command_feedback.status.STATUS_COMMAND_OVERRIDDEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "Arm_command | COMMAND_SYNCHRONIZED_FEEDBACK_SET | STATUS_COMMAND_OVERRIDDEN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.status.value = (
            arm_command_feedback.status.STATUS_COMMAND_TIMED_OUT
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "Arm_command | COMMAND_SYNCHRONIZED_FEEDBACK_SET | STATUS_COMMAND_TIMED_OUT",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.status.value = (
            arm_command_feedback.status.STATUS_ROBOT_FROZEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "Arm_command | COMMAND_SYNCHRONIZED_FEEDBACK_SET | STATUS_ROBOT_FROZEN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.status.value = (
            arm_command_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "Arm_command | COMMAND_SYNCHRONIZED_FEEDBACK_SET | STATUS_INCOMPATIBLE_HARDWARE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.status.value = (
            arm_command_feedback.status.STATUS_PROCESSING
        )

        """ Testing arm cartesian feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_cartesian_feedback.status.value = (
            arm_command_feedback.feedback.arm_cartesian_feedback.status.STATUS_TRAJECTORY_COMPLETE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET | STATUS_TRAJECTORY_COMPLETE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_cartesian_feedback.status.value = (
            arm_command_feedback.feedback.arm_cartesian_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_cartesian_feedback.status.value = (
            arm_command_feedback.feedback.arm_cartesian_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_cartesian_feedback.status.value = (
            arm_command_feedback.feedback.arm_cartesian_feedback.status.STATUS_TRAJECTORY_CANCELLED
        )

        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET | STATUS_TRAJECTORY_CANCELLED",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_cartesian_feedback.status.value = (
            arm_command_feedback.feedback.arm_cartesian_feedback.status.STATUS_TRAJECTORY_STALLED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET | STATUS_TRAJECTORY_STALLED",
        )

        """ Testing arm joint move feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_joint_move_feedback.status.value = (
            arm_command_feedback.feedback.arm_joint_move_feedback.status.STATUS_COMPLETE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET | STATUS_COMPLETE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_joint_move_feedback.status.value = (
            arm_command_feedback.feedback.arm_joint_move_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_joint_move_feedback.status.value = (
            arm_command_feedback.feedback.arm_joint_move_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_joint_move_feedback.status.value = (
            arm_command_feedback.feedback.arm_joint_move_feedback.status.STATUS_STALLED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET | STATUS_STALLED",
        )

        """ Testing named arm position feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.named_arm_position_feedback.status.value = (
            arm_command_feedback.feedback.named_arm_position_feedback.status.STATUS_COMPLETE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET | STATUS_COMPLETE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.named_arm_position_feedback.status.value = (
            arm_command_feedback.feedback.named_arm_position_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.named_arm_position_feedback.status.value = (
            arm_command_feedback.feedback.named_arm_position_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.named_arm_position_feedback.status.value = (
            arm_command_feedback.feedback.named_arm_position_feedback.status.STATUS_STALLED_HOLDING_ITEM
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET | STATUS_STALLED_HOLDING_ITEM",
        )

        """ Testing arm velocity feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET
        )
        # Arm velocity commands do not provide feedback therefore we should get a success
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_VELOCITY_FEEDBACK_SET",
        )

        """ Testing arm gaze feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_GAZE_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_gaze_feedback.status.value = (
            arm_command_feedback.feedback.arm_gaze_feedback.status.STATUS_TRAJECTORY_COMPLETE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_GAZE_FEEDBACK_SET | STATUS_TRAJECTORY_COMPLETE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_gaze_feedback.status.value = (
            arm_command_feedback.feedback.arm_gaze_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_GAZE_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_gaze_feedback.status.value = (
            arm_command_feedback.feedback.arm_gaze_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_GAZE_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_gaze_feedback.status.value = (
            arm_command_feedback.feedback.arm_gaze_feedback.status.STATUS_TOOL_TRAJECTORY_STALLED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_GAZE_FEEDBACK_SET | STATUS_TOOL_TRAJECTORY_STALLED",
        )

        """ Testing arm stop feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_STOP_FEEDBACK_SET
        )
        # Arm stop commands do not provide feedback therefore we should get a success
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_STOP_FEEDBACK_SET",
        )

        """ Testing arm drag feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_DRAG_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_drag_feedback.status.value = (
            arm_command_feedback.feedback.arm_drag_feedback.status.STATUS_DRAGGING
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_DRAG_FEEDBACK_SET | STATUS_DRAGGING",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_drag_feedback.status.value = (
            arm_command_feedback.feedback.arm_drag_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_DRAG_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_drag_feedback.status.value = (
            arm_command_feedback.feedback.arm_drag_feedback.status.STATUS_GRASP_FAILED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_DRAG_FEEDBACK_SET | STATUS_GRASP_FAILED",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_drag_feedback.status.value = (
            arm_command_feedback.feedback.arm_drag_feedback.status.STATUS_OTHER_FAILURE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_DRAG_FEEDBACK_SET | STATUS_OTHER_FAILURE",
        )

        """ Testing arm impedance feedback """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = (
            arm_command_feedback.feedback.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET
        )
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_impedance_feedback.status.value = (
            arm_command_feedback.feedback.arm_impedance_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_impedance_feedback.status.value = (
            arm_command_feedback.feedback.arm_impedance_feedback.status.STATUS_TRAJECTORY_COMPLETE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET | STATUS_TRAJECTORY_COMPLETE",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_impedance_feedback.status.value = (
            arm_command_feedback.feedback.arm_impedance_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_impedance_feedback.status.value = (
            arm_command_feedback.feedback.arm_impedance_feedback.status.STATUS_TRAJECTORY_CANCELLED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET | STATUS_TRAJECTORY_CANCELLED",
        )

        feedback.command.synchronized_feedback.arm_command_feedback.feedback.arm_impedance_feedback.status.value = (
            arm_command_feedback.feedback.arm_impedance_feedback.status.STATUS_TRAJECTORY_STALLED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET | STATUS_TRAJECTORY_STALLED",
        )

        """ Testing unknown arm command """
        feedback.command.synchronized_feedback.arm_command_feedback.feedback.feedback_choice = FEEDBACK_INVALID
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "Arm Command: FEEDBACK_INVALID",
        )

        """ Testing mobility commands """
        mobility_feedback = RobotCommandFeedback().command.synchronized_feedback.mobility_command_feedback

        feedback.command.synchronized_feedback.has_field = (
            feedback.command.synchronized_feedback.MOBILITY_COMMAND_FEEDBACK_FIELD_SET
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.status.value = (
            mobility_feedback.status.STATUS_COMMAND_OVERRIDDEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "MOBILITY | COMMAND_SYNCHRONIZED_FEEDBACK | STATUS_COMMAND_OVERRIDDEN",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.status.value = (
            mobility_feedback.status.STATUS_COMMAND_TIMED_OUT
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "MOBILITY | COMMAND_SYNCHRONIZED_FEEDBACK | | STATUS_COMMAND_TIMED_OUT",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.status.value = (
            mobility_feedback.status.STATUS_ROBOT_FROZEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "MOBILITY | COMMAND_SYNCHRONIZED_FEEDBACK | | STATUS_ROBOT_FROZEN",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.status.value = (
            mobility_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "MOBILITY | COMMAND_SYNCHRONIZED_FEEDBACK | | STATUS_INCOMPATIBLE_HARDWARE",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.status.value = (
            mobility_feedback.status.STATUS_PROCESSING
        )

        """ Testing se2 trajectory feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
            mobility_feedback.feedback.se2_trajectory_feedback.status.STATUS_AT_GOAL
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET | STATUS_AT_GOAL",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
            mobility_feedback.feedback.se2_trajectory_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
            mobility_feedback.feedback.se2_trajectory_feedback.status.STATUS_NEAR_GOAL
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET | STATUS_NEAR_GOAL",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.status.value = (
            mobility_feedback.feedback.se2_trajectory_feedback.status.STATUS_GOING_TO_GOAL
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET | STATUS_GOING_TO_GOAL",
        )

        """ Testing se2 velocity feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET
        )
        # Planar velocity commands provide no feedback, therefore expect SUCCESS
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_SE2_VELOCITY_FEEDBACK_SET",
        )

        """ Testing sit feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_SIT_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.sit_feedback.status.value = (
            mobility_feedback.feedback.sit_feedback.status.STATUS_IS_SITTING
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_SIT_FEEDBACK_SET | STATUS_IS_SITTING",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.sit_feedback.status.value = (
            mobility_feedback.feedback.sit_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SIT_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.sit_feedback.status.value = (
            mobility_feedback.feedback.sit_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_SIT_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        """ Testing stand feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_STAND_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stand_feedback.status.value = (
            mobility_feedback.feedback.stand_feedback.status.STATUS_IS_STANDING
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_STAND_FEEDBACK_SET | STATUS_IS_STANDING",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stand_feedback.status.value = (
            mobility_feedback.feedback.stand_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_STAND_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stand_feedback.status.value = (
            mobility_feedback.feedback.stand_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_STAND_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        """ Testing stance feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_STANCE_FEEDBACK_SET
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stance_feedback.status.value = (
            mobility_feedback.feedback.stance_feedback.status.STATUS_TOO_FAR_AWAY
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "FEEDBACK_STANCE_FEEDBACK_SET | STATUS_TOO_FAR_AWAY",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stance_feedback.status.value = (
            mobility_feedback.feedback.stance_feedback.status.STATUS_STANCED
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_STANCE_FEEDBACK_SET | STATUS_STANCED",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stance_feedback.status.value = (
            mobility_feedback.feedback.stance_feedback.status.STATUS_GOING_TO_STANCE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_STANCE_FEEDBACK_SET | STATUS_GOING_TO_STANCE",
        )

        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.stance_feedback.status.value = (
            mobility_feedback.feedback.stance_feedback.status.STATUS_UNKNOWN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "FEEDBACK_STANCE_FEEDBACK_SET | STATUS_UNKNOWN",
        )

        """ Testing stop feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_STOP_FEEDBACK_SET
        )
        # Stop commands provide no feedback, therefore expect SUCCESS
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_STOP_FEEDBACK_SET",
        )

        """ Testing stop feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET
        )
        # follow arm commands provide no feedback, therefore expect SUCCESS
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "FEEDBACK_FOLLOW_ARM_FEEDBACK_SET",
        )

        """ Testing stop feedback """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = (
            mobility_feedback.feedback.FEEDBACK_NOT_SET
        )
        # mobility command feedback is not set, this could be caused by a command that finishes and resets the feedback status.
        # because of this case, it will return success as long as no other synchronous commands are run afterwards.
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "MOBILITY | FEEDBACK_NOT_SET",
        )

        """ Testing unknown command """
        feedback.command.synchronized_feedback.mobility_command_feedback.feedback.feedback_choice = FEEDBACK_INVALID
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "MOBILITY| FEEDBACK_INVALID",
        )

        """ Testing Gripper commands """
        gripper_feedback = RobotCommandFeedback().command.synchronized_feedback.gripper_command_feedback

        feedback.command.synchronized_feedback.has_field = (
            feedback.command.synchronized_feedback.GRIPPER_COMMAND_FEEDBACK_FIELD_SET
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.status.value = (
            gripper_feedback.status.STATUS_COMMAND_OVERRIDDEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "GRIPPER | COMMAND_SYNCHRONIZED_FEEDBACK | STATUS_COMMAND_OVERRIDDEN",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.status.value = (
            gripper_feedback.status.STATUS_COMMAND_TIMED_OUT
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "GRIPPER | COMMAND_SYNCHRONIZED_FEEDBACK | STATUS_COMMAND_TIMED_OUT",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.status.value = (
            gripper_feedback.status.STATUS_ROBOT_FROZEN
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "GRIPPER | COMMAND_SYNCHRONIZED_FEEDBACK | STATUS_ROBOT_FROZEN",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.status.value = (
            gripper_feedback.status.STATUS_INCOMPATIBLE_HARDWARE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.FAILED,
            "GRIPPER | COMMAND_SYNCHRONIZED_FEEDBACK | STATUS_INCOMPATIBLE_HARDWARE",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.status.value = (
            gripper_feedback.status.STATUS_PROCESSING
        )

        """ Testing Claw Gripper feedback """
        feedback.command.synchronized_feedback.gripper_command_feedback.command.command_choice = (
            gripper_feedback.command.COMMAND_CLAW_GRIPPER_FEEDBACK_SET
        )
        feedback.command.synchronized_feedback.gripper_command_feedback.command.claw_gripper_feedback.status.value = (
            gripper_feedback.command.claw_gripper_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.command.claw_gripper_feedback.status.value = (
            gripper_feedback.command.claw_gripper_feedback.status.STATUS_IN_PROGRESS
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | STATUS_IN_PROGRESS",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.command.claw_gripper_feedback.status.value = (
            gripper_feedback.command.claw_gripper_feedback.status.STATUS_AT_GOAL
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | STATUS_AT_GOAL",
        )

        feedback.command.synchronized_feedback.gripper_command_feedback.command.claw_gripper_feedback.status.value = (
            gripper_feedback.command.claw_gripper_feedback.status.STATUS_APPLYING_FORCE
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.SUCCESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | STATUS_APPLYING_FORCE",
        )

        """ Testing unknown gripper command """
        feedback.command.synchronized_feedback.gripper_command_feedback.command.command_choice = FEEDBACK_INVALID
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | FEEDBACK_INVALID",
        )

        """ Testing unknown robot command type """
        feedback.command.command_choice = FEEDBACK_INVALID
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(dummy_command, feedback),
            GoalResponse.IN_PROGRESS,
            "COMMAND_CLAW_GRIPPER_FEEDBACK_SET | FEEDBACK_INVALID",
        )

        """ Testing battery change pose corner case """
        feedback.command.command_choice = feedback.command.COMMAND_FULL_BODY_FEEDBACK_SET
        feedback.command.full_body_feedback.status.value = (
            feedback.command.full_body_feedback.status.STATUS_COMMAND_OVERRIDDEN
        )

        overriden_command = RobotCommand()
        overriden_command.command.full_body_command.command.which = (
            overriden_command.command.full_body_command.command.COMMAND_BATTERY_CHANGE_POSE_REQUEST_SET
        )
        self.assertEqual(
            self.spot_ros2._robot_command_goal_complete(overriden_command, feedback),
            GoalResponse.SUCCESS,
            "NO FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET | STATUS_COMMAND_OVERRIDDEN",
        )

    def test_frame_prefix_from_spot_name(self) -> None:
        self.assertEqual(self.spot_ros2.frame_prefix, "Mock_spot/", "spot_name not used in frame_prefix")


if __name__ == "__main__":
    unittest.main()
