#!/usr/bin/env python
# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.
import argparse
import time
from typing import Optional

import synchros2.process as ros_process
from sensor_msgs.msg import JointState
from synchros2.futures import unwrap_future
from synchros2.node import Node
from synchros2.subscription import Subscription
from synchros2.utilities import namespace_with

from spot_msgs.msg import JointCommand

# maximum and minimum joint angles in radians.
GRIPPER_OPEN_ANGLE = -1.57
GRIPPER_CLOSE_ANGLE = 0.0
GRIPPER_JOINT_NAME = "arm_f1x"


class ExampleGripperStreaming:
    def __init__(self, node: Node, robot_name: Optional[str] = None) -> None:
        """Initialize the example."""
        self._node = node
        self._logger = self._node.get_logger()
        self._robot_name = robot_name
        self._command_pub = self._node.create_publisher(
            JointCommand, namespace_with(self._robot_name, "spot_joint_controller/joint_commands"), 10
        )
        self._joint_command = JointCommand()
        self._joint_command.name = [namespace_with(self._robot_name, GRIPPER_JOINT_NAME)]
        self._joint_command.k_q_p = [16.0]
        self._joint_command.k_qd_p = [0.32]
        self._joint_angles = Subscription(JointState, namespace_with(self._robot_name, "low_level/joint_states"))

    def get_gripper_joint_angle(self) -> float:
        """Get the current gripper joint angle from the joint state topic"""
        joint_state = unwrap_future(self._joint_angles.update, timeout_sec=5.0)
        gripper_index = joint_state.name.index(namespace_with(self._robot_name, GRIPPER_JOINT_NAME))
        gripper_position = joint_state.position[gripper_index]
        return gripper_position

    def get_k_q_p(self) -> float:
        """Get the current k_q_p command for the gripper."""
        return self._joint_command.k_q_p[0]

    def get_k_qd_p(self) -> float:
        """Get the current k_qd_p command for the gripper."""
        return self._joint_command.k_qd_p[0]

    def set_gains(self, k_q_p: float, k_qd_p: float) -> None:
        """Set the k_q_p and k_qd_p commands for the gripper."""
        self._logger.info(f"Setting k_q_p={k_q_p} and k_qd_p={k_qd_p}")
        self._joint_command.k_q_p = [k_q_p]
        self._joint_command.k_qd_p = [k_qd_p]

    def move_gripper(self, goal_joint_angle: float) -> None:
        """Command the gripper to a given joint angle by streaming a command."""
        self._joint_command.position = [goal_joint_angle]
        self._command_pub.publish(self._joint_command)

    def open_and_close(self, duration_sec: float = 1.0, frequency_hz: float = 50.0) -> None:
        """Open and close the gripper by streaming position commands.

        Args:
            duration_sec (float): Duration in seconds of each open and close movement
            frequency_hz (int): Frequency in Hz of the command publish rate.
        """
        current_gripper_angle = self.get_gripper_joint_angle()
        npoints = int(duration_sec * frequency_hz)
        dt = 1.0 / frequency_hz
        step_size_open = (GRIPPER_OPEN_ANGLE - current_gripper_angle) / npoints
        self._logger.info("Opening...")
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(current_gripper_angle + i * step_size_open))
            time.sleep(dt)
        self._logger.info("Closing...")
        step_size_close = (GRIPPER_OPEN_ANGLE - GRIPPER_CLOSE_ANGLE) / npoints
        for i in range(npoints):
            self.move_gripper(goal_joint_angle=(GRIPPER_OPEN_ANGLE - i * step_size_close))
            time.sleep(dt)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, help="Namespace the driver is in", default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    """Run the example."""
    example = ExampleGripperStreaming(node=main.node, robot_name=args.robot)
    example.open_and_close()
    while True:
        # Get gains to try out from the user.
        k_q_p_str = input(f"Select a k_q_p gain (current value: {example.get_k_q_p()}): ")
        k_qd_p_str = input(f"Select a k_qd_p gain (current value: {example.get_k_qd_p()}): ")
        try:
            # clamp k_q_p from 0->50 (nominally 16). O will result in no motion and anything over ~20 looks the same.
            k_q_p = max(0.0, min(float(k_q_p_str), 50.0))
            # clamp k_qd_p from 0->4 (nominally 0.32). Gripper makes weird noises when this is >= 5...
            k_qd_p = max(0.0, min(float(k_qd_p_str), 4.0))
            example.set_gains(k_q_p, k_qd_p)
        except ValueError:
            print("Your inputs could not be converted to floats -- keeping the same gains.")
        # open and close the gripper with the new gains set.
        example.open_and_close()


if __name__ == "__main__":
    main()
