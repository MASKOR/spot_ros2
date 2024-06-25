// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <bosdyn/math/proto_math.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>
#include <builtin_interfaces/msg/duration.hpp>
#include <optional>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/conversions/common_conversions.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/conversions/time.hpp>

namespace spot_ros2 {

spot_msgs::msg::BatteryStateArray getBatteryStates(const ::bosdyn::api::RobotState& robot_state,
                                                   const google::protobuf::Duration& clock_skew) {
  spot_msgs::msg::BatteryStateArray battery_states;

  for (const auto& battery : robot_state.battery_states()) {
    spot_msgs::msg::BatteryState battery_state;

    battery_state.header.stamp = robotTimeToLocalTime(battery.timestamp(), clock_skew);
    battery_state.identifier = battery.identifier();
    battery_state.charge_percentage = battery.charge_percentage().value();
    battery_state.estimated_runtime = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                                          .sec(battery.estimated_runtime().seconds())
                                          .nanosec(battery.estimated_runtime().nanos());
    battery_state.current = battery.current().value();
    battery_state.voltage = battery.voltage().value();
    for (const auto& temp : battery.temperatures()) {
      battery_state.temperatures.push_back(temp);
    }
    battery_state.status = battery.status();
    battery_states.battery_states.push_back(battery_state);
  }

  return battery_states;
}

spot_msgs::msg::WiFiState getWifiState(const ::bosdyn::api::RobotState& robot_state) {
  spot_msgs::msg::WiFiState wifi_state;

  for (const auto& comm_state : robot_state.comms_states()) {
    if (comm_state.has_wifi_state()) {
      wifi_state.current_mode = comm_state.wifi_state().current_mode();
      wifi_state.essid = comm_state.wifi_state().essid();
    }
  }

  return wifi_state;
}

spot_msgs::msg::FootStateArray getFootState(const ::bosdyn::api::RobotState& robot_state) {
  spot_msgs::msg::FootStateArray foot_states;

  for (const auto& foot : robot_state.foot_state()) {
    spot_msgs::msg::FootState foot_state;
    convertToRos(foot.foot_position_rt_body(), foot_state.foot_position_rt_body);
    foot_state.contact = foot.contact();
    foot_states.states.push_back(foot_state);
  }

  return foot_states;
}

spot_msgs::msg::EStopStateArray getEstopStates(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew) {
  spot_msgs::msg::EStopStateArray estop_states;

  for (const auto& estop : robot_state.estop_states()) {
    spot_msgs::msg::EStopState estop_state;
    estop_state.header.stamp = robotTimeToLocalTime(estop.timestamp(), clock_skew);
    estop_state.name = estop.name();
    estop_state.type = estop.type();
    estop_state.state = estop.state();
    estop_state.state_description = estop.state_description();
    estop_states.estop_states.push_back(estop_state);
  }

  return estop_states;
}

std::optional<sensor_msgs::msg::JointState> getJointStates(const ::bosdyn::api::RobotState& robot_state,
                                                           const google::protobuf::Duration& clock_skew,
                                                           const std::string& prefix) {
  if (!robot_state.has_kinematic_state()) {
    return std::nullopt;
  }

  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = robotTimeToLocalTime(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);

  for (const auto& joint : robot_state.kinematic_state().joint_states()) {
    const auto joint_name = prefix + kFriendlyJointNames.at(joint.name());
    joint_states.name.push_back(joint_name);
    joint_states.position.push_back(joint.position().value());
    joint_states.velocity.push_back(joint.velocity().value());
    joint_states.effort.push_back(joint.load().value());
  }

  return joint_states;
}

std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::RobotState& robot_state,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              [[maybe_unused]] const std::string& preferred_base_frame_id) {
  if (!robot_state.has_kinematic_state() || !robot_state.kinematic_state().has_transforms_snapshot()) {
    return std::nullopt;
  }

  return getTf(robot_state.kinematic_state().transforms_snapshot(),
               robot_state.kinematic_state().acquisition_timestamp(), clock_skew, prefix, preferred_base_frame_id);
}

std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::FrameTreeSnapshot& frame_tree_snapshot,
                                              const google::protobuf::Timestamp& timestamp_robot,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& preferred_base_frame_id,
                                              const std::set<std::string, std::less<>>& frames_to_ignore) {
  if (frame_tree_snapshot.child_to_parent_edge_map().empty()) {
    return std::nullopt;
  }

  const auto timestamp_local = robotTimeToLocalTime(timestamp_robot, clock_skew);

  tf2_msgs::msg::TFMessage tf_msg;
  for (const auto& [frame_id, transform] : frame_tree_snapshot.child_to_parent_edge_map()) {
    // In Spot's FrameTreeSnapshot, a frame without a parent is a root frame.
    // In TF, root frames are expressed by publishing a transform whose parent frame ID is not the child frame ID of any
    // other transform. To satisfy this requirement, do not publish frames from the frame tree snapshot if they do not
    // have a parent frame ID.
    if (transform.parent_frame_name().empty()) {
      continue;
    }

    // If this frame is in the list of frames to ignore, skip it.
    if (frames_to_ignore.find(frame_id) != frames_to_ignore.end()) {
      continue;
    }

    // These frames are duplicates of arm_link_wr1 (published with robot state) and shouldn't be added to the TF tree!
    if ((frame_id == "arm0.link_wr1") || (frame_id == "link_wr1")) {
      continue;
    }

    const auto parent_frame_name = transform.parent_frame_name().find('/') == std::string::npos
                                       ? prefix + transform.parent_frame_name()
                                       : transform.parent_frame_name();
    const auto frame_name = frame_id.find('/') == std::string::npos ? prefix + frame_id : frame_id;

    // set target frame(preferred odom frame) as the root node in tf tree
    if (preferred_base_frame_id == frame_name) {
      tf_msg.transforms.push_back(
          toTransformStamped(~(transform.parent_tform_child()), frame_name, parent_frame_name, timestamp_local));
    } else {
      tf_msg.transforms.push_back(
          toTransformStamped(transform.parent_tform_child(), parent_frame_name, frame_name, timestamp_local));
    }
  }
  return tf_msg;
}

std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> getOdomTwist(
    const ::bosdyn::api::RobotState& robot_state, const google::protobuf::Duration& clock_skew) {
  if (!robot_state.has_kinematic_state() || !robot_state.kinematic_state().has_velocity_of_body_in_odom() ||
      !robot_state.kinematic_state().has_transforms_snapshot()) {
    return std::nullopt;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped odom_twist_msg;
  // TODO(schornakj): need to add the frame ID here?
  const auto& kinematic_state = robot_state.kinematic_state();
  odom_twist_msg.header.stamp = spot_ros2::robotTimeToLocalTime(kinematic_state.acquisition_timestamp(), clock_skew);
  const bosdyn::api::SE3Velocity velocity_of_body_in_odom = kinematic_state.velocity_of_body_in_odom();
  // This now needs to be converted to velocity of body in body frame in order to follow ROS conventions.
  ::bosdyn::api::SE3Velocity velocity_of_body_in_body;
  const std::string odom_frame_name = "odom";
  const std::string body_frame_name = "body";
  const bool success =
      ::bosdyn::api::ExpressVelocityInNewFrame(kinematic_state.transforms_snapshot(), odom_frame_name, body_frame_name,
                                               velocity_of_body_in_odom, &velocity_of_body_in_body);
  if (!success) {
    return std::nullopt;
  }
  convertToRos(velocity_of_body_in_body, odom_twist_msg.twist.twist);
  return odom_twist_msg;
}

std::optional<nav_msgs::msg::Odometry> getOdom(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                               bool is_using_vision) {
  if (!robot_state.has_kinematic_state() || !robot_state.kinematic_state().has_acquisition_timestamp() ||
      !robot_state.kinematic_state().has_transforms_snapshot() ||
      !robot_state.kinematic_state().has_velocity_of_body_in_odom()) {
    return std::nullopt;
  }

  const auto odom_twist = getOdomTwist(robot_state, clock_skew);
  if (!odom_twist) {
    return std::nullopt;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.twist = odom_twist.value().twist;

  const auto& kinematic_state = robot_state.kinematic_state();
  odom_msg.header.stamp = robotTimeToLocalTime(kinematic_state.acquisition_timestamp(), clock_skew);

  ::bosdyn::api::SE3Pose tf_body_pose;
  if (is_using_vision) {
    if (!::bosdyn::api::GetWorldTformBody(kinematic_state.transforms_snapshot(), &tf_body_pose)) {
      return std::nullopt;
    }
    odom_msg.header.frame_id = prefix + "vision";
  } else {
    if (!::bosdyn::api::GetOdomTformBody(kinematic_state.transforms_snapshot(), &tf_body_pose)) {
      return std::nullopt;
    }
    odom_msg.header.frame_id = prefix + "odom";
  }
  convertToRos(tf_body_pose, odom_msg.pose.pose);
  odom_msg.child_frame_id = prefix + "body";
  return odom_msg;
}

std::optional<spot_msgs::msg::PowerState> getPowerState(const ::bosdyn::api::RobotState& robot_state,
                                                        const google::protobuf::Duration& clock_skew) {
  if (!robot_state.has_power_state()) {
    return std::nullopt;
  }

  spot_msgs::msg::PowerState power_state;

  power_state.header.stamp = robotTimeToLocalTime(robot_state.power_state().timestamp(), clock_skew);
  power_state.motor_power_state = robot_state.power_state().motor_power_state();
  power_state.shore_power_state = robot_state.power_state().shore_power_state();
  power_state.locomotion_charge_percentage = robot_state.power_state().locomotion_charge_percentage().value();
  power_state.locomotion_estimated_runtime =
      builtin_interfaces::build<builtin_interfaces::msg::Duration>()
          .sec(robot_state.power_state().locomotion_estimated_runtime().seconds())
          .nanosec(robot_state.power_state().locomotion_estimated_runtime().nanos());

  return power_state;
}

std::optional<spot_msgs::msg::SystemFaultState> getSystemFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                    const google::protobuf::Duration& clock_skew) {
  if (!robot_state.has_system_fault_state()) {
    return std::nullopt;
  }
  const auto create_fault_message = [&clock_skew](const ::bosdyn::api::SystemFault& fault) {
    spot_msgs::msg::SystemFault fault_msg;
    fault_msg.name = fault.name();
    fault_msg.header.stamp = robotTimeToLocalTime(fault.onset_timestamp(), clock_skew);
    fault_msg.duration = builtin_interfaces::build<builtin_interfaces::msg::Duration>()
                             .sec(fault.duration().seconds())
                             .nanosec(fault.duration().nanos());
    fault_msg.code = fault.code();
    fault_msg.uid = fault.uid();
    fault_msg.error_message = fault.error_message();
    for (const auto& attr : fault.attributes()) {
      fault_msg.attributes.push_back(attr);
    }
    fault_msg.severity = fault.severity();
    return fault_msg;
  };

  spot_msgs::msg::SystemFaultState system_fault_state;
  for (const auto& fault : robot_state.system_fault_state().faults()) {
    system_fault_state.faults.push_back(create_fault_message(fault));
  }
  for (const auto& fault : robot_state.system_fault_state().historical_faults()) {
    system_fault_state.historical_faults.push_back(create_fault_message(fault));
  }
  return system_fault_state;
}

std::optional<bosdyn_api_msgs::msg::ManipulatorState> getManipulatorState(
    const ::bosdyn::api::RobotState& robot_state) {
  using ManipulatorState = bosdyn_api_msgs::msg::ManipulatorState;

  if (!robot_state.has_manipulator_state()) {
    return std::nullopt;
  }

  const auto& manipulator_state = robot_state.manipulator_state();

  ManipulatorState manipulator_state_msg;
  manipulator_state_msg.has_field = 0u;

  manipulator_state_msg.gripper_open_percentage = manipulator_state.gripper_open_percentage();
  manipulator_state_msg.is_gripper_holding_item = manipulator_state.is_gripper_holding_item();

  if (manipulator_state.has_estimated_end_effector_force_in_hand()) {
    convertToRos(manipulator_state.estimated_end_effector_force_in_hand(),
                 manipulator_state_msg.estimated_end_effector_force_in_hand);
    manipulator_state_msg.has_field |= ManipulatorState::ESTIMATED_END_EFFECTOR_FORCE_IN_HAND_FIELD_SET;
  }

  manipulator_state_msg.stow_state.value = manipulator_state.stow_state();

  if (manipulator_state.has_velocity_of_hand_in_vision()) {
    convertToRos(manipulator_state.velocity_of_hand_in_vision(), manipulator_state_msg.velocity_of_hand_in_vision);
    manipulator_state_msg.has_field |= ManipulatorState::VELOCITY_OF_HAND_IN_VISION_FIELD_SET;
  }

  if (manipulator_state.has_velocity_of_hand_in_odom()) {
    convertToRos(manipulator_state.velocity_of_hand_in_odom(), manipulator_state_msg.velocity_of_hand_in_odom);
    manipulator_state_msg.has_field |= ManipulatorState::VELOCITY_OF_HAND_IN_ODOM_FIELD_SET;
  }

  manipulator_state_msg.carry_state.value = manipulator_state.carry_state();
  return manipulator_state_msg;
}

std::optional<geometry_msgs::msg::Vector3Stamped> getEndEffectorForce(const ::bosdyn::api::RobotState& robot_state,
                                                                      const google::protobuf::Duration& clock_skew,
                                                                      const std::string& prefix) {
  if (!robot_state.has_manipulator_state()) {
    return std::nullopt;
  }
  geometry_msgs::msg::Vector3Stamped force;
  force.header.stamp = robotTimeToLocalTime(robot_state.kinematic_state().acquisition_timestamp(), clock_skew);
  force.header.frame_id = prefix + "hand";
  convertToRos(robot_state.manipulator_state().estimated_end_effector_force_in_hand(), force.vector);
  return force;
}

std::optional<spot_msgs::msg::BehaviorFaultState> getBehaviorFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                        const google::protobuf::Duration& clock_skew) {
  if (!robot_state.has_behavior_fault_state()) {
    return std::nullopt;
  }

  spot_msgs::msg::BehaviorFaultState behavior_fault_msgs;

  for (const auto& fault : robot_state.behavior_fault_state().faults()) {
    spot_msgs::msg::BehaviorFault fault_msg;
    fault_msg.behavior_fault_id = fault.behavior_fault_id();
    fault_msg.header.stamp = robotTimeToLocalTime(fault.onset_timestamp(), clock_skew);
    fault_msg.cause = fault.cause();
    fault_msg.status = fault.status();
    behavior_fault_msgs.faults.push_back(fault_msg);
  }
  return behavior_fault_msgs;
}

}  // namespace spot_ros2
