# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
Module containing test fixtures.
Pytest automatically discovers all fixtures defined in the file "conftest.py".
"""

# We disable Pylint warnings for all Protobuf files which contain objects with
# dynamically added member attributes.
# pylint: disable=no-member

# When a test needs a fixture, it must specify the fixture name as a parameter.
# In doings so, Pylint raises an incorrect warning about a name being redefined,
# warning that we want disabled.
# pylint: disable=redefined-outer-name

import os
import pathlib
import tempfile
import typing

import bdai_ros2_wrappers.scope as ros_scope
import domain_coordinator
import grpc
import launch
import launch.actions
import launch.substitutions
import launch_pytest
import launch_pytest.actions
import launch_ros
import launch_ros.substitutions
import pytest
import rclpy
import yaml
from bdai_ros2_wrappers.scope import ROSAwareScope
from bosdyn.api.power_pb2 import PowerCommandRequest, PowerCommandResponse, PowerCommandStatus
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from bosdyn.api.robot_state_pb2 import PowerState

import spot_wrapper.testing
from spot_driver.spot_ros2 import SpotROS
from spot_wrapper.testing.fixtures import SpotFixture
from spot_wrapper.testing.mocks import MockSpot

TEST_PATH = pathlib.Path(__file__).parent
os.environ.update(
    {
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTRTPS_DEFAULT_PROFILES_FILE": str(TEST_PATH.parent / "profile.xml"),
    }
)


# pylint: disable=invalid-name,unused-argument
@spot_wrapper.testing.fixture
class simple_spot(MockSpot):
    """
    This is a factory that returns an instance of the class MockSpot,
    served by a local GRPC server.
    When the class is disposed, the GRPC server is shut down.
    The MockSpot and the GRPC server are used to handle calls to a simulated
    Spot robot.
    """

    def PowerCommand(self, request: PowerCommandRequest, context: grpc.ServicerContext) -> PowerCommandResponse:
        """
        Dummy implementation of the PowerCommand command.  This is necessary because many calls in
        SpotWrapper (including _robot_command) are decorated with a call to power on first.  Without
        this none of those calls will work.
        """
        response = PowerCommandResponse()
        if request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS:
            self.robot_state.power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
            response.status = PowerCommandStatus.STATUS_SUCCESS
        elif request.request == PowerCommandRequest.Request.REQUEST_OFF_MOTORS:
            self.robot_state.power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF
            response.status = PowerCommandStatus.STATUS_SUCCESS
        else:
            response.status = PowerCommandStatus.STATUS_INTERNAL_ERROR
        return response


# pylint: enable=invalid-name,unused-argument


@pytest.fixture
def domain_id() -> typing.Iterator[int]:
    with domain_coordinator.domain_id() as domain_id:
        yield domain_id


@pytest.fixture
def ros(simple_spot: SpotFixture, domain_id: int) -> typing.Iterator[ROSAwareScope]:
    """
    This method is a generator function that returns a different ROS2 scope
    each time it is invoked, to handle a ROS2 context lifecycle.
    """
    with ros_scope.top(global_=True, namespace=simple_spot.api.name, domain_id=domain_id) as top:
        yield top


@pytest.fixture
def spot_node(ros: ROSAwareScope, simple_spot: SpotFixture) -> typing.Iterator[SpotROS]:
    """
    This method is a generator function that returns a different SpotROS
    node each time it is invoked.
    The node is assigned and executed inside the given ROS context, and it is
    destroyed when the context goes out of scope.
    """
    parameter_overrides = [
        rclpy.parameter.Parameter("username", rclpy.Parameter.Type.STRING, "spot"),
        rclpy.parameter.Parameter("password", rclpy.Parameter.Type.STRING, "spot"),
        rclpy.parameter.Parameter("hostname", rclpy.Parameter.Type.STRING, simple_spot.address),
        rclpy.parameter.Parameter("port", rclpy.Parameter.Type.INTEGER, simple_spot.port),
        rclpy.parameter.Parameter("certificate", rclpy.Parameter.Type.STRING, str(simple_spot.certificate_path)),
        rclpy.parameter.Parameter("spot_name", rclpy.Parameter.Type.STRING, simple_spot.api.name),
        # disable camera publishing for now
        rclpy.parameter.Parameter("publish_rgb", rclpy.Parameter.Type.BOOL, False),
        rclpy.parameter.Parameter("publish_depth", rclpy.Parameter.Type.BOOL, False),
    ]
    with ros.managed(SpotROS, parameter_overrides=parameter_overrides) as node:
        try:
            yield node
        finally:
            # succeed sit command on destruction
            response = RobotCommandResponse()
            response.status = RobotCommandResponse.Status.STATUS_OK  # pylint: disable=no-member
            simple_spot.api.RobotCommand.future.returns(response)


@launch_pytest.fixture
def spot_graph_description(simple_spot: SpotFixture, domain_id: int) -> typing.Iterator[launch.LaunchDescription]:
    with tempfile.NamedTemporaryFile(mode="w", suffix="config.yaml") as temp:
        data = {
            "username": "user",
            "password": "pass",
            "hostname": simple_spot.address,
            "port": simple_spot.port,
            "certificate": str(simple_spot.certificate_path),
            "spot_name": simple_spot.api.name,
            "rgb_cameras": False,
        }
        yaml.dump({"/**": {"ros__parameters": data}}, temp.file)
        temp.file.close()

        yield launch.LaunchDescription(
            [
                launch.actions.SetEnvironmentVariable("ROS_DOMAIN_ID", str(domain_id)),
                launch.actions.SetEnvironmentVariable(
                    name="RMW_IMPLEMENTATION", value=os.environ["RMW_IMPLEMENTATION"]
                ),
                launch.actions.SetEnvironmentVariable(
                    name="FASTRTPS_DEFAULT_PROFILES_FILE", value=os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]
                ),
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        launch.substitutions.PathJoinSubstitution(
                            [
                                launch_ros.substitutions.FindPackageShare("spot_driver"),
                                "launch",
                                "spot_driver.launch.py",
                            ]
                        )
                    ),
                    launch_arguments=[("config_file", temp.file.name), ("spot_name", simple_spot.api.name)],
                ),
                launch_pytest.actions.ReadyToTest(),
            ],
        )


def pytest_configure() -> None:
    pytest.spot_graph_description = spot_graph_description
