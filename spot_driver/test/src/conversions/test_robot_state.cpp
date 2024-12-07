// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <gmock/gmock-matchers.h>
#include <gmock/gmock-more-matchers.h>
#include <gmock/gmock.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/map.h>
#include <google/protobuf/timestamp.pb.h>
#include <bosdyn_api_msgs/msg/manipulator_state_carry_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iterator>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/matchers.hpp>
#include <spot_driver/robot_state_test_tools.hpp>
#include <spot_msgs/msg/battery_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/e_stop_state.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>

namespace {
using ::testing::AllOf;
using ::testing::Contains;
using ::testing::DoubleEq;
using ::testing::Eq;
using ::testing::Field;
using ::testing::IsEmpty;
using ::testing::IsFalse;
using ::testing::IsTrue;
using ::testing::Not;
using ::testing::SizeIs;
using ::testing::StrEq;
using ::testing::UnorderedElementsAre;
}  // namespace

namespace spot_ros2::test {
TEST(RobotStateConversions, TestGetBatteryStates) {
  // GIVEN a clock skew and a RobotState containing a BatteryState, a WifiState, FootState, and an EstopState
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(5);
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(15);
  ::bosdyn::api::RobotState robot_state;
  auto* battery_state = robot_state.add_battery_states();
  battery_state->CopyFrom(
      createBatteryState("test_battery", timestamp, 50, 10, 12, 80.0,
                         ::bosdyn::api::BatteryState_Status::BatteryState_Status_STATUS_DISCHARGING));

  // WHEN we create a BatteryStateArray ROS message from the RobotState
  const spot_msgs::msg::BatteryStateArray out = getBatteryStates(robot_state, clock_skew);

  // THEN the ROS message contains the same values as were set in the input RobotState
  ASSERT_THAT(out.battery_states, SizeIs(1));
  const auto& first_battery_state = out.battery_states.at(0);
  EXPECT_THAT(first_battery_state.header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
  EXPECT_THAT(first_battery_state.identifier, StrEq("test_battery"));
  EXPECT_THAT(first_battery_state.charge_percentage, DoubleEq(50.0));
  EXPECT_THAT(first_battery_state.current, DoubleEq(10.0));
  EXPECT_THAT(first_battery_state.voltage, DoubleEq(12.0));
  ASSERT_THAT(first_battery_state.temperatures, SizeIs(1UL));
  EXPECT_THAT(first_battery_state.temperatures.at(0), DoubleEq(80.0));
}

TEST(RobotStateConversions, TestGetWifiState) {
  // GIVEN a RobotState which contains a valid CommsState field
  constexpr auto input_mode = ::bosdyn::api::WiFiState_Mode::WiFiState_Mode_MODE_CLIENT;
  constexpr auto input_essid = "some_value";
  ::bosdyn::api::RobotState robot_state;
  auto* comms_state = robot_state.add_comms_states();
  comms_state->mutable_wifi_state()->CopyFrom(createWifiState(input_mode, input_essid));

  // WHEN we create a WiFiState ROS message from the RobotState
  const spot_msgs::msg::WiFiState out = getWifiState(robot_state);

  // THEN the ROS message contains the same values for the ESSID and mode fields as were set in the input RobotState
  EXPECT_THAT(out.current_mode, Eq(input_mode));
  EXPECT_THAT(out.essid, StrEq(input_essid));
}

TEST(RobotStateConversions, TestGetFootState) {
  // GIVEN a RobotState that contains four unique foot states
  ::bosdyn::api::RobotState robot_state;
  setFootState(robot_state.add_foot_state(), 1.0, 2.0, 3.0, ::bosdyn::api::FootState::CONTACT_MADE);
  setFootState(robot_state.add_foot_state(), 4.0, 5.0, 6.0, ::bosdyn::api::FootState::CONTACT_LOST);
  setFootState(robot_state.add_foot_state(), 7.0, 8.0, 9.0, ::bosdyn::api::FootState::CONTACT_MADE);
  setFootState(robot_state.add_foot_state(), 10.0, 11.0, 12.0, ::bosdyn::api::FootState::CONTACT_LOST);

  // WHEN we create a FootStateArray ROS message from the RobotState
  const auto out = getFootState(robot_state);

  // THEN the output message contains four foot states
  ASSERT_THAT(out.states, SizeIs(4));

  // THEN each output foot state matches the corresponding input, and is listed in the same order
  const auto& first_foot_state = out.states.at(0);
  EXPECT_THAT(first_foot_state.contact, Eq(spot_msgs::msg::FootState::CONTACT_MADE));
  EXPECT_THAT(first_foot_state.foot_position_rt_body, GeometryMsgsPointEq(1.0, 2.0, 3.0));

  const auto& second_foot_state = out.states.at(1);
  EXPECT_THAT(second_foot_state.contact, Eq(spot_msgs::msg::FootState::CONTACT_LOST));
  EXPECT_THAT(second_foot_state.foot_position_rt_body, GeometryMsgsPointEq(4.0, 5.0, 6.0));

  const auto& third_foot_state = out.states.at(2);
  EXPECT_THAT(third_foot_state.contact, Eq(spot_msgs::msg::FootState::CONTACT_MADE));
  EXPECT_THAT(third_foot_state.foot_position_rt_body, GeometryMsgsPointEq(7.0, 8.0, 9.0));

  const auto& fourth_foot_state = out.states.at(3);
  EXPECT_THAT(fourth_foot_state.contact, Eq(spot_msgs::msg::FootState::CONTACT_LOST));
  EXPECT_THAT(fourth_foot_state.foot_position_rt_body, GeometryMsgsPointEq(10.0, 11.0, 12.0));
}

TEST(RobotStateConversions, TestGetFootStateNoFootStates) {
  // GIVEN a RobotState that does not contain foot states
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a FootStateArray ROS message from the RobotState
  const auto out = getFootState(robot_state);

  // THEN the output message contains an empty array
  ASSERT_THAT(out.states, IsEmpty());
}

TEST(RobotStateConversions, TestGetEStopStates) {
  // GIVEN a RobotState that contains two different EStopStates
  ::bosdyn::api::RobotState robot_state;

  google::protobuf::Timestamp timestamp_first;
  timestamp_first.set_seconds(13);
  timestamp_first.set_nanos(0);
  ::bosdyn::api::EStopState estop_state_first;
  estop_state_first.mutable_timestamp()->CopyFrom(timestamp_first);
  estop_state_first.set_name("estop_name");
  estop_state_first.set_type(::bosdyn::api::EStopState_Type::EStopState_Type_TYPE_HARDWARE);
  estop_state_first.set_state(::bosdyn::api::EStopState_State::EStopState_State_STATE_ESTOPPED);
  estop_state_first.set_state_description("some_text");
  robot_state.mutable_estop_states()->Add(std::move(estop_state_first));

  google::protobuf::Timestamp timestamp_second;
  timestamp_second.set_seconds(12);
  timestamp_second.set_nanos(0);
  ::bosdyn::api::EStopState estop_state_second;
  estop_state_second.mutable_timestamp()->CopyFrom(timestamp_second);
  estop_state_second.set_name("second_estop_name");
  estop_state_second.set_type(::bosdyn::api::EStopState_Type::EStopState_Type_TYPE_SOFTWARE);
  estop_state_second.set_state(::bosdyn::api::EStopState_State::EStopState_State_STATE_NOT_ESTOPPED);
  estop_state_second.set_state_description("other_text");
  robot_state.mutable_estop_states()->Add(std::move(estop_state_second));

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create an EStopStatesArray ROS message from the RobotState
  const auto out = getEstopStates(robot_state, clock_skew);

  // THEN the output message contains both estop states
  ASSERT_THAT(out.estop_states, SizeIs(2));

  // THEN each output estop state contains equivalent data to its corresponding input
  EXPECT_THAT(out.estop_states, EStopStatesContains("estop_name", spot_msgs::msg::EStopState::TYPE_HARDWARE,
                                                    spot_msgs::msg::EStopState::STATE_ESTOPPED, "some_text",
                                                    timestamp_first, clock_skew));
  EXPECT_THAT(out.estop_states, EStopStatesContains("second_estop_name", spot_msgs::msg::EStopState::TYPE_SOFTWARE,
                                                    spot_msgs::msg::EStopState::STATE_NOT_ESTOPPED, "other_text",
                                                    timestamp_second, clock_skew));
}

TEST(RobotStateConversions, TestGetJointStates) {
  // GIVEN a RobotState containing two unique joint states
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(15);
  timestamp.set_nanos(0);
  robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(timestamp);
  setJointState(robot_state.mutable_kinematic_state()->add_joint_states(), "fl.hx", 0.1, 0.2, 0.3, 0.4);
  setJointState(robot_state.mutable_kinematic_state()->add_joint_states(), "arm0.wr0", 0.5, 0.6, 0.7, 0.8);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a prefix
  constexpr auto prefix = "my_prefix/";

  // WHEN we create an JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN a message is created
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN each vector element of the joint state contains a number of elements matching the number of input joint states
  ASSERT_THAT(out->name, SizeIs(2));
  ASSERT_THAT(out->position, SizeIs(2));
  ASSERT_THAT(out->velocity, SizeIs(2));
  ASSERT_THAT(out->effort, SizeIs(2));

  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));

  // THEN the joint names were converted to the "friendly" names used in the URDF
  ASSERT_THAT(out->name, UnorderedElementsAre("my_prefix/front_left_hip_x", "my_prefix/arm_wr0"));

  // THEN the position, velocity, and effort corresponding to each named joint match the input joint states
  const std::size_t index1 =
      std::distance(out->name.cbegin(), std::find(out->name.cbegin(), out->name.cend(), "my_prefix/front_left_hip_x"));
  EXPECT_THAT(out->position.at(index1), DoubleEq(0.1));
  EXPECT_THAT(out->velocity.at(index1), DoubleEq(0.2));
  EXPECT_THAT(out->effort.at(index1), DoubleEq(0.4));

  const std::size_t index2 =
      std::distance(out->name.cbegin(), std::find(out->name.cbegin(), out->name.cend(), "my_prefix/arm_wr0"));
  EXPECT_THAT(out->position.at(index2), DoubleEq(0.5));
  EXPECT_THAT(out->velocity.at(index2), DoubleEq(0.6));
  EXPECT_THAT(out->effort.at(index2), DoubleEq(0.8));
}

TEST(RobotStateConversions, TestGetJointStatesNoJointStates) {
  // GIVEN a RobotState that does not contain any joint states, but does contains some other kinematic state info
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(15);
  timestamp.set_nanos(0);
  robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(timestamp);

  // GIVEN some nominal clock skew and prefix
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  constexpr auto prefix = "prefix/";

  // WHEN we create a JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN the message is created and contains a timestamp, but does not contain any joint state data
  ASSERT_THAT(out.has_value(), IsTrue());
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
  EXPECT_THAT(out->name, IsEmpty());
  EXPECT_THAT(out->position, IsEmpty());
  EXPECT_THAT(out->velocity, IsEmpty());
  EXPECT_THAT(out->effort, IsEmpty());
}

TEST(RobotStateConversions, TestGetJointStatesNoKinematicState) {
  // GIVEN a RobotState that does not contain any kinematic state data whatsoever
  ::bosdyn::api::RobotState robot_state;

  // GIVEN some nominal clock skew and prefix
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  constexpr auto prefix = "prefix/";

  // WHEN we attempt to create a JointState ROS message from the RobotState
  const auto out = getJointStates(robot_state, clock_skew, prefix);

  // THEN no ROS message is output
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetTf) {
  // GIVEN a RobotState containing a valid transform snapshot
  ::bosdyn::api::RobotState robot_state;
  // GIVEN a nonzero acquisition timestamp
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TF tree from the RobotState
  const auto out = getTf(robot_state, clock_skew, "prefix/", "odom");
  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the tree contains one frame
  ASSERT_THAT(out->transforms, SizeIs(1));

  // THEN this frame matches the transform from the odom frame to the body frame
  const auto& transform = out->transforms.at(0);
  EXPECT_THAT(transform.header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
  EXPECT_THAT(transform.header.frame_id, StrEq("prefix/odom"));
  EXPECT_THAT(transform.child_frame_id, StrEq("prefix/body"));
  EXPECT_THAT(transform.transform, GeometryMsgsTransformEq(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0));
}

TEST(RobotStateConversions, TestGetTfInverted) {
  // GIVEN a RobotState containing a valid transform snapshot
  ::bosdyn::api::RobotState robot_state;
  // GIVEN a nonzero acquisition timestamp
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TF tree from the RobotState
  const auto out = getTf(robot_state, clock_skew, "prefix/", "prefix/body");
  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the tree contains one frame
  ASSERT_THAT(out->transforms, SizeIs(1));

  // THEN this frame is the inverse of the transform from the odom frame to the body frame
  const auto& transform = out->transforms.at(0);
  EXPECT_THAT(transform.header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
  EXPECT_THAT(transform.header.frame_id, StrEq("prefix/body"));
  EXPECT_THAT(transform.child_frame_id, StrEq("prefix/odom"));
  EXPECT_THAT(transform.transform, GeometryMsgsTransformEq(-1.0, -2.0, -3.0, 1.0, 0.0, 0.0, 0.0));
}

TEST(RobotStateConversions, TestGetOdomTwistNoTransformsSnapshot) {
  // GIVEN a RobotState that contains info about the velocity of the body in the odom frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);

  auto* velocity_linear = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(1.0);
  velocity_linear->set_y(2.0);
  velocity_linear->set_z(3.0);
  auto* velocity_angular = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_angular();
  velocity_angular->set_x(1.0);
  velocity_angular->set_y(2.0);
  velocity_angular->set_z(3.0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TwistWithCovarianceStamped ROS message
  const bool using_vision = false;
  const auto out = getOdomTwist(robot_state, clock_skew, using_vision);

  // THEN this fails as there is no TransformsSnapshot
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetOdomTwistBodyAtOdom) {
  // GIVEN a RobotState that contains info about the velocity of the body in the odom frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);

  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at the odom frame
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 0.0, 0.0, 0.0, 1.0,
               0.0, 0.0, 0.0);

  auto* velocity_linear = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(1.0);
  velocity_linear->set_y(2.0);
  velocity_linear->set_z(3.0);
  auto* velocity_angular = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_angular();
  velocity_angular->set_x(1.0);
  velocity_angular->set_y(2.0);
  velocity_angular->set_z(3.0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TwistWithCovarianceStamped ROS message
  const bool using_vision = false;
  const auto out = getOdomTwist(robot_state, clock_skew, using_vision);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the output twist matches the velocity in the robot state.
  EXPECT_THAT(out->twist.twist, GeometryMsgsTwistEq(1.0, 2.0, 3.0, 1.0, 2.0, 3.0));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
}

TEST(RobotStateConversions, TestGetOdomTwistLinearTransformationLinearVelocity) {
  // GIVEN a RobotState that contains info about the velocity of the body in the odom frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);

  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is linearly translated 2m forward in the x direction
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 2.0, 0.0, 0.0, 1.0,
               0.0, 0.0, 0.0);

  auto* velocity_linear = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(1.0);
  velocity_linear->set_y(0.0);
  velocity_linear->set_z(0.0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TwistWithCovarianceStamped ROS message
  const bool using_vision = false;
  const auto out = getOdomTwist(robot_state, clock_skew, using_vision);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the output twist matches the velocity in the robot state.
  EXPECT_THAT(out->twist.twist, GeometryMsgsTwistEq(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
}

TEST(RobotStateConversions, TestGetOdomTwistAngularTransformationLinearVelocity) {
  // GIVEN a RobotState that contains info about the velocity of the body in the odom frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);

  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is rotated by 180 degrees from the odom frame (quaternion 0 0 0 1)
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0);

  auto* velocity_linear = robot_state.mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(1.0);
  velocity_linear->set_y(0.0);
  velocity_linear->set_z(0.0);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a TwistWithCovarianceStamped ROS message
  const bool using_vision = false;
  const auto out = getOdomTwist(robot_state, clock_skew, using_vision);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the output twist matches the velocity in the robot state.
  EXPECT_THAT(out->twist.twist, GeometryMsgsTwistEq(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
}

TEST(RobotStateConversions, TestGetOdomTwistNoBodyVelocityInRobotState) {
  // GIVEN a RobotState where there is some kinematic state info but no info about the velocity of the body in the odom
  // frame
  ::bosdyn::api::RobotState robot_state;

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we attempt to create a TwistWithCovarianceStamped ROS message
  const bool using_vision = false;
  const auto out = getOdomTwist(robot_state, clock_skew, using_vision);

  // THEN no message is output
  ASSERT_THAT(out.has_value(), IsFalse());
}

// TODO(khughes): Fix this test
TEST(RobotStateConversions, TestGetOdomInOdomFrame) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState
  ::bosdyn::api::RobotState robot_state;
  // GIVEN a nonzero acquisition timestamp
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  // GIVEN the odom frame is the root of the frame tree
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  // GIVEN the body frame is at a nonzero pose relative to the odom frame
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  // GIVEN the body is moving at a nonzero velocity relative to the odom frame
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  // GIVEN the frame tree snapshot is valid
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we create a nav_msgs::msg::Odometry ROS message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the parent and child frame IDs of the Odometry pose refer to the correct frames and prepend the prefix to the
  // frame IDs
  EXPECT_THAT(out->header.frame_id, StrEq("prefix/odom"));
  EXPECT_THAT(out->child_frame_id, StrEq("prefix/body"));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));

  // THEN the Odometry ROS message contains the same pose and twist data as the RobotState
  EXPECT_THAT(out->pose.pose, GeometryMsgsPoseEq(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0));
  EXPECT_THAT(out->twist.twist, GeometryMsgsTwistEq(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
}

// TODO(khughes): Fix this test
TEST(RobotStateConversions, TestGetOdomInVisionFrame) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that is fully populated with the required data about the robot's pose and velocity in the vision
  // frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  addBodyVelocityVision(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "vision");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "vision", 1.0, 2.0, 3.0,
               1.0, 0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we create a nav_msgs::msg::Odometry ROS message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", true);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the parent and child frame IDs of the Odometry pose refer to the correct frames and prepend the prefix to the
  // frame IDs
  EXPECT_THAT(out->header.frame_id, StrEq("prefix/vision"));
  EXPECT_THAT(out->child_frame_id, StrEq("prefix/body"));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));

  // THEN the Odometry ROS message contains the same pose and twist data as the RobotState
  EXPECT_THAT(out->pose.pose, GeometryMsgsPoseEq(1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0));
  EXPECT_THAT(out->twist.twist, GeometryMsgsTwistEq(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
}

TEST(RobotStateConversions, TestGetOdomMissingAcquisitionTimestamp) {
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that contains some odom info, but does not contain an acquisition timestamp
  ::bosdyn::api::RobotState robot_state;
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetOdomMissingBodyVelocityOdom) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that does not contain info about the body velocity
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  addRootFrame(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "odom");
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "odom", 1.0, 2.0, 3.0, 1.0,
               0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::VALID));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetOdomMissingTransforms) {
  // GIVEN a RobotState that does not contain a transform snapshot
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  addBodyVelocityOdom(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", false);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetOdomInvalidTransformSnapshot) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that contains a transform snapshot which is invalid because it does not have a root frame
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(99);
  timestamp.set_nanos(0);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);
  addBodyVelocityVision(robot_state.mutable_kinematic_state(), 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  addTransform(robot_state.mutable_kinematic_state()->mutable_transforms_snapshot(), "body", "vision", 1.0, 2.0, 3.0,
               1.0, 0.0, 0.0, 0.0);
  ASSERT_THAT(bosdyn::api::ValidateFrameTreeSnapshot(robot_state.kinematic_state().transforms_snapshot()),
              Eq(::bosdyn::api::ValidateFrameTreeSnapshotStatus::UNKNOWN_PARENT_FRAME_NAME));

  // WHEN we try to create an Odometry message from the RobotState
  const auto out = getOdom(robot_state, clock_skew, "prefix/", true);

  // THEN this does not succeed
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetPowerState) {
  // GIVEN a RobotState that contains a fully-populated power state
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_power_state()->set_robot_power_state(
      ::bosdyn::api::PowerState_RobotPowerState::PowerState_RobotPowerState_ROBOT_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_motor_power_state(
      ::bosdyn::api::PowerState_MotorPowerState::PowerState_MotorPowerState_MOTOR_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_shore_power_state(
      ::bosdyn::api::PowerState_ShorePowerState::PowerState_ShorePowerState_SHORE_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_payload_ports_power_state(
      ::bosdyn::api::PowerState_PayloadPortsPowerState::PowerState_PayloadPortsPowerState_PAYLOAD_PORTS_POWER_STATE_ON);
  robot_state.mutable_power_state()->set_wifi_radio_power_state(
      ::bosdyn::api::PowerState_WifiRadioPowerState::PowerState_WifiRadioPowerState_WIFI_RADIO_POWER_STATE_ON);
  robot_state.mutable_power_state()->mutable_locomotion_charge_percentage()->set_value(75.0);
  google::protobuf::Duration estimated_runtime;
  estimated_runtime.set_seconds(255);
  estimated_runtime.set_nanos(0);
  robot_state.mutable_power_state()->mutable_locomotion_estimated_runtime()->CopyFrom(estimated_runtime);
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(60);
  robot_state.mutable_power_state()->mutable_timestamp()->CopyFrom(timestamp);

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a PowerState ROS message
  const auto out = getPowerState(robot_state, clock_skew);

  // THEN a message is output
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the fields in the output message match their corresponding inputs
  EXPECT_THAT(out->motor_power_state, Eq(spot_msgs::msg::PowerState::STATE_ON));
  EXPECT_THAT(out->shore_power_state, Eq(spot_msgs::msg::PowerState::STATE_ON_SHORE_POWER));
  EXPECT_THAT(out->locomotion_charge_percentage, DoubleEq(75.0));
  EXPECT_THAT(out->locomotion_estimated_runtime, DurationEq(estimated_runtime));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(robot_state.power_state().timestamp(), clock_skew));
}

TEST(RobotStateConversions, TestGetPowerStateNoPowerState) {
  // GIVEN a RobotState that does not contain a power state
  ::bosdyn::api::RobotState robot_state;

  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // WHEN we create a PowerState ROS message
  const auto out = getPowerState(robot_state, clock_skew);

  // THEN no message is output
  ASSERT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetSystemFaultState) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN the robot state contains two system faults, which occur at different timestamps and contain different data
  ::bosdyn::api::RobotState robot_state;
  google::protobuf::Timestamp timestamp1;
  timestamp1.set_seconds(60);
  google::protobuf::Duration duration1;
  duration1.set_seconds(15);
  duration1.set_nanos(0);
  appendSystemFault(robot_state.mutable_system_fault_state(), timestamp1, duration1, "fault1", 19, 3, "battery is low",
                    {"robot", "battery"}, ::bosdyn::api::SystemFault_Severity::SystemFault_Severity_SEVERITY_WARN);

  google::protobuf::Timestamp timestamp2;
  timestamp2.set_seconds(75);
  google::protobuf::Duration duration2;
  duration2.set_seconds(0);
  duration2.set_nanos(0);
  appendSystemFault(robot_state.mutable_system_fault_state(), timestamp2, duration2, "fault2", 55, 9,
                    "robot has departed from this plane of reality", {"robot"},
                    ::bosdyn::api::SystemFault_Severity::SystemFault_Severity_SEVERITY_CRITICAL);

  // WHEN we create a SystemFaultState ROS message from the Robot State
  const auto out = getSystemFaultState(robot_state, clock_skew);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN the output message contains two faults
  // THEN each of the output faults contains fields that match the input
  // THEN the clock skew is correctly applied
  EXPECT_THAT(out->faults,
              UnorderedElementsAre(
                  SystemFaultEq(timestamp1, clock_skew, duration1, "fault1", 3UL, 19, "battery is low",
                                std::vector<std::string>{"robot", "battery"}),
                  SystemFaultEq(timestamp2, clock_skew, duration2, "fault2", 9UL, 55,
                                "robot has departed from this plane of reality", std::vector<std::string>{"robot"})));
}

TEST(RobotStateConversions, TestGetSystemFaultStateNoFault) {
  // GIVEN some nominal clock skew
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState that does not contain any system faults
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a SystemFaultState ROS message from the Robot State
  const auto out = getSystemFaultState(robot_state, clock_skew);

  // THEN no ROS message is output
  EXPECT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetManipulatorState) {
  // GIVEN a RobotState which has all manipulator state fields populated
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_manipulator_state()->set_gripper_open_percentage(50.0);
  robot_state.mutable_manipulator_state()->set_is_gripper_holding_item(true);
  ::bosdyn::api::Vec3 force_in_hand;
  force_in_hand.set_x(1.0);
  force_in_hand.set_y(2.0);
  force_in_hand.set_z(3.0);
  robot_state.mutable_manipulator_state()->mutable_estimated_end_effector_force_in_hand()->CopyFrom(force_in_hand);

  ::bosdyn::api::SE3Velocity velocity_hand_vision;
  velocity_hand_vision.mutable_linear()->set_x(1.0);
  velocity_hand_vision.mutable_linear()->set_y(2.0);
  velocity_hand_vision.mutable_linear()->set_z(3.0);
  velocity_hand_vision.mutable_angular()->set_x(4.0);
  velocity_hand_vision.mutable_angular()->set_y(5.0);
  velocity_hand_vision.mutable_angular()->set_z(6.0);
  robot_state.mutable_manipulator_state()->mutable_velocity_of_hand_in_vision()->CopyFrom(velocity_hand_vision);

  ::bosdyn::api::SE3Velocity velocity_hand_odom;
  velocity_hand_odom.mutable_linear()->set_x(1.0);
  velocity_hand_odom.mutable_linear()->set_y(2.0);
  velocity_hand_odom.mutable_linear()->set_z(3.0);
  velocity_hand_odom.mutable_angular()->set_x(4.0);
  velocity_hand_odom.mutable_angular()->set_y(5.0);
  velocity_hand_odom.mutable_angular()->set_z(6.0);
  robot_state.mutable_manipulator_state()->mutable_velocity_of_hand_in_odom()->CopyFrom(velocity_hand_odom);

  robot_state.mutable_manipulator_state()->set_carry_state(
      ::bosdyn::api::ManipulatorState_CarryState::ManipulatorState_CarryState_CARRY_STATE_NOT_CARRIABLE);
  robot_state.mutable_manipulator_state()->set_stow_state(
      ::bosdyn::api::ManipulatorState_StowState::ManipulatorState_StowState_STOWSTATE_DEPLOYED);

  // WHEN we use the RobotState to create a ManipulatorState ROS message
  const auto out = getManipulatorState(robot_state);

  // THEN this succeeds
  ASSERT_THAT(out.has_value(), IsTrue());

  // THEN all the output fields are set, and contain the same values as the inputs
  EXPECT_THAT(out->gripper_open_percentage, DoubleEq(50.0));
  EXPECT_THAT(out->is_gripper_holding_item, IsTrue());

  using ManipulatorStateCarryState = bosdyn_api_msgs::msg::ManipulatorStateCarryState;
  using ManipulatorStateStowState = bosdyn_api_msgs::msg::ManipulatorStateStowState;
  EXPECT_THAT(out->carry_state.value, Eq(ManipulatorStateCarryState::CARRY_STATE_NOT_CARRIABLE));
  EXPECT_THAT(out->stow_state.value, Eq(ManipulatorStateStowState::STOWSTATE_DEPLOYED));

  using ManipulatorState = bosdyn_api_msgs::msg::ManipulatorState;
  EXPECT_THAT(out->has_field, HasEnabledBit(ManipulatorState::ESTIMATED_END_EFFECTOR_FORCE_IN_HAND_FIELD_SET));
  EXPECT_THAT(out->estimated_end_effector_force_in_hand,
              GeometryMsgsVector3Eq(force_in_hand.x(), force_in_hand.y(), force_in_hand.z()));

  EXPECT_THAT(out->has_field, HasEnabledBit(ManipulatorState::VELOCITY_OF_HAND_IN_VISION_FIELD_SET));
  EXPECT_THAT(out->velocity_of_hand_in_vision,
              GeometryMsgsTwistEq(velocity_hand_vision.linear().x(), velocity_hand_vision.linear().y(),
                                  velocity_hand_vision.linear().z(), velocity_hand_vision.angular().x(),
                                  velocity_hand_vision.angular().y(), velocity_hand_vision.angular().z()));

  EXPECT_THAT(out->has_field, HasEnabledBit(ManipulatorState::VELOCITY_OF_HAND_IN_ODOM_FIELD_SET));
  EXPECT_THAT(out->velocity_of_hand_in_odom,
              GeometryMsgsTwistEq(velocity_hand_odom.linear().x(), velocity_hand_odom.linear().y(),
                                  velocity_hand_odom.linear().z(), velocity_hand_odom.angular().x(),
                                  velocity_hand_odom.angular().y(), velocity_hand_odom.angular().z()));
}

TEST(RobotStateConversions, TestGetManipulatorStateNoManipulatorState) {
  // GIVEN an empty RobotState
  ::bosdyn::api::RobotState robot_state;

  // WHEN we call getEndEffectorForce()
  const auto out = getManipulatorState(robot_state);
  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetManipulatorStateNoForceInHand) {
  // GIVEN an empty RobotState
  ::bosdyn::api::RobotState robot_state;
  robot_state.mutable_manipulator_state()->set_is_gripper_holding_item(true);
  robot_state.mutable_manipulator_state()->clear_estimated_end_effector_force_in_hand();

  // WHEN we call getEndEffectorForce()
  const auto out = getManipulatorState(robot_state);
  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), IsTrue());

  using ManipulatorState = bosdyn_api_msgs::msg::ManipulatorState;
  EXPECT_THAT(out->has_field, Not(HasEnabledBit(ManipulatorState::ESTIMATED_END_EFFECTOR_FORCE_IN_HAND_FIELD_SET)));
}

TEST(RobotStateConversions, TestGetEndEffectorForce) {
  // GIVEN nominal timestamps and nonzero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(10);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  const auto* const prefix = "prefix/";

  // GIVEN a RobotState containing the estimated end effector force
  ::bosdyn::api::RobotState robot_state;
  ::bosdyn::api::Vec3 force;
  force.set_x(1.0);
  force.set_y(2.0);
  force.set_z(3.0);
  robot_state.mutable_manipulator_state()->mutable_estimated_end_effector_force_in_hand()->CopyFrom(force);
  robot_state.mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(timestamp);
  addAcquisitionTimestamp(robot_state.mutable_kinematic_state(), timestamp);

  // WHEN we create a Vector3Stamped ROS message to represent the estimated end effector force
  const auto out = getEndEffectorForce(robot_state, clock_skew, prefix);

  // THEN the fields match
  ASSERT_THAT(out.has_value(), IsTrue());
  EXPECT_THAT(out->header.frame_id, StrEq("prefix/hand"));
  EXPECT_THAT(out->header.stamp.sec, Eq(9));
  EXPECT_THAT(out->header, ClockSkewIsAppliedToHeader(timestamp, clock_skew));
  EXPECT_THAT(out->vector, GeometryMsgsVector3Eq(force.x(), force.y(), force.z()));
}

TEST(RobotStateConversions, TestGetEndEffectorForceNoEndEffectorForce) {
  // GIVEN an empty RobotState
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);
  const auto* const prefix = "prefix/";
  ::bosdyn::api::RobotState robot_state;

  // WHEN we call getEndEffectorForce()
  const auto out = getEndEffectorForce(robot_state, clock_skew, prefix);

  // THEN the conversion does not succeed
  EXPECT_THAT(out.has_value(), IsFalse());
}

TEST(RobotStateConversions, TestGetBehaviorFaultState) {
  // GIVEN nominal timestamps and nonzero clock skew
  google::protobuf::Timestamp timestamp;
  timestamp.set_seconds(10);
  timestamp.set_nanos(0);
  google::protobuf::Duration clock_skew;
  clock_skew.set_seconds(1);

  // GIVEN a RobotState containing two faults
  ::bosdyn::api::RobotState robot_state;
  auto* fault_state_first = robot_state.mutable_behavior_fault_state()->add_faults();
  fault_state_first->set_behavior_fault_id(11);
  fault_state_first->set_status(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_CLEARABLE);
  fault_state_first->set_cause(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_HARDWARE);
  fault_state_first->mutable_onset_timestamp()->CopyFrom(timestamp);
  auto* fault_state_second = robot_state.mutable_behavior_fault_state()->add_faults();
  fault_state_second->set_behavior_fault_id(12);
  fault_state_second->set_status(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_UNCLEARABLE);
  fault_state_second->set_cause(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_FALL);
  fault_state_second->mutable_onset_timestamp()->CopyFrom(timestamp);

  // WHEN we create a BehaviorFaultState ROS message from the robot state
  const auto out = getBehaviorFaultState(robot_state, clock_skew);

  // THEN the output optional contains a message
  ASSERT_THAT(out.has_value(), IsTrue());
  // THEN the message contains two faults
  ASSERT_THAT(out->faults, SizeIs(2));
  // THEN the first fault matches the first one added to the RobotState, and the clock skew is applied correctly
  const auto first_fault = out->faults.at(0);
  EXPECT_THAT(first_fault.behavior_fault_id, Eq(11U));
  EXPECT_THAT(first_fault.status, Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_CLEARABLE));
  EXPECT_THAT(first_fault.cause, Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_HARDWARE));
  EXPECT_THAT(first_fault.header, ClockSkewIsAppliedToHeader(fault_state_first->onset_timestamp(), clock_skew));
  // THEN the second fault matches the second one added to the RobotState, and the clock skew is applied correctly
  const auto second_fault = out->faults.at(1);
  EXPECT_THAT(second_fault.behavior_fault_id, Eq(12U));
  EXPECT_THAT(second_fault.status, Eq(::bosdyn::api::BehaviorFault_Status::BehaviorFault_Status_STATUS_UNCLEARABLE));
  EXPECT_THAT(second_fault.cause, Eq(::bosdyn::api::BehaviorFault_Cause::BehaviorFault_Cause_CAUSE_FALL));
  EXPECT_THAT(second_fault.header, ClockSkewIsAppliedToHeader(fault_state_second->onset_timestamp(), clock_skew));
}

TEST(RobotStateConversions, TestGetBehaviorFaultStateNoFaults) {
  // GIVEN a robot state that does not contain any behavior fault states
  google::protobuf::Duration clock_skew;
  ::bosdyn::api::RobotState robot_state;

  // WHEN we create a BehaviorFaultState ROS message from the robot state
  const auto out = getBehaviorFaultState(robot_state, clock_skew);

  // THEN the optional which wraps the output ROS message is set to nullopt
  EXPECT_THAT(out.has_value(), IsFalse());
}
}  // namespace spot_ros2::test
