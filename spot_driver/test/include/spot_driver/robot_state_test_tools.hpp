// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/geometry.pb.h>
#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn/math/frame_helpers.h>
#include <google/protobuf/duration.pb.h>
#include <google/protobuf/map.h>
#include <google/protobuf/timestamp.pb.h>
#include <bosdyn_api_msgs/msg/manipulator_state_carry_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_msgs/msg/battery_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>

#include <string>
#include <vector>

namespace spot_ros2::test {
inline ::bosdyn::api::BatteryState createBatteryState(const std::string& id,
                                                      const google::protobuf::Timestamp& timestamp, uint32_t percentage,
                                                      uint32_t current, uint32_t voltage, double temperature,
                                                      ::bosdyn::api::BatteryState_Status status) {
  ::bosdyn::api::BatteryState out;
  out.set_identifier(id);
  out.mutable_timestamp()->CopyFrom(timestamp);
  out.set_status(status);
  out.add_temperatures(temperature);
  out.mutable_charge_percentage()->set_value(percentage);
  out.mutable_current()->set_value(current);
  out.mutable_voltage()->set_value(voltage);
  return out;
}

inline ::bosdyn::api::WiFiState createWifiState(::bosdyn::api::WiFiState_Mode mode, const std::string& essid) {
  ::bosdyn::api::WiFiState out;
  out.set_current_mode(mode);
  *out.mutable_essid() = essid;
  return out;
}

inline void appendSystemFault(::bosdyn::api::SystemFaultState* mutable_fault_state,
                              const google::protobuf::Timestamp& onset_timestamp,
                              const google::protobuf::Duration& duration, const std::string& name, const int32_t code,
                              const uint64_t uid, const std::string& error_message,
                              const std::vector<std::string>& attributes,
                              const ::bosdyn::api::SystemFault_Severity& severity) {
  auto* fault = mutable_fault_state->add_faults();
  fault->mutable_onset_timestamp()->CopyFrom(onset_timestamp);
  fault->mutable_duration()->CopyFrom(duration);
  fault->set_name(name);
  fault->set_code(code);
  fault->set_uid(uid);
  fault->set_error_message(error_message);
  *fault->mutable_attributes() = {attributes.cbegin(), attributes.cend()};
  fault->set_severity(severity);
}

inline void setFootState(::bosdyn::api::FootState* foot_state, const double x, const double y, const double z,
                         const ::bosdyn::api::FootState_Contact contact) {
  foot_state->mutable_foot_position_rt_body()->set_x(x);
  foot_state->mutable_foot_position_rt_body()->set_y(y);
  foot_state->mutable_foot_position_rt_body()->set_z(z);
  foot_state->set_contact(contact);
}

inline void setJointState(::bosdyn::api::JointState* joint_state, const std::string& name, const double position,
                          const double velocity, const double acceleration, const double load) {
  *joint_state->mutable_name() = name;
  joint_state->mutable_position()->set_value(position);
  joint_state->mutable_velocity()->set_value(velocity);
  joint_state->mutable_acceleration()->set_value(acceleration);
  joint_state->mutable_load()->set_value(load);
}

inline void addTransform(::bosdyn::api::FrameTreeSnapshot* mutable_frame_tree_snapshot, const std::string& child_name,
                         const std::string& parent_name, const double x, const double y, const double z,
                         const double qw, const double qx, const double qy, const double qz) {
  auto* edge_map = mutable_frame_tree_snapshot->mutable_child_to_parent_edge_map();
  ::bosdyn::api::FrameTreeSnapshot_ParentEdge edge;
  *edge.mutable_parent_frame_name() = parent_name;
  auto* pos = edge.mutable_parent_tform_child()->mutable_position();
  pos->set_x(x);
  pos->set_y(y);
  pos->set_z(z);
  auto* rot = edge.mutable_parent_tform_child()->mutable_rotation();
  rot->set_w(qw);
  rot->set_x(qx);
  rot->set_y(qy);
  rot->set_z(qz);
  edge_map->insert(google::protobuf::MapPair{child_name, edge});
}

inline void addRootFrame(::bosdyn::api::FrameTreeSnapshot* mutable_frame_tree_snapshot, const std::string& root_frame) {
  auto* edge_map = mutable_frame_tree_snapshot->mutable_child_to_parent_edge_map();
  ::bosdyn::api::FrameTreeSnapshot_ParentEdge root_edge;
  edge_map->insert(google::protobuf::MapPair{root_frame, root_edge});
}

inline void addBodyVelocityOdom(::bosdyn::api::KinematicState* mutable_kinematic_state, double x, double y, double z,
                                double rx, double ry, double rz) {
  auto* velocity_linear = mutable_kinematic_state->mutable_velocity_of_body_in_odom()->mutable_linear();
  velocity_linear->set_x(x);
  velocity_linear->set_y(y);
  velocity_linear->set_z(z);
  auto* velocity_angular = mutable_kinematic_state->mutable_velocity_of_body_in_odom()->mutable_angular();
  velocity_angular->set_x(rx);
  velocity_angular->set_y(ry);
  velocity_angular->set_z(rz);
}

inline void addBodyVelocityVision(::bosdyn::api::KinematicState* mutable_kinematic_state, double x, double y, double z,
                                  double rx, double ry, double rz) {
  auto* velocity_linear = mutable_kinematic_state->mutable_velocity_of_body_in_vision()->mutable_linear();
  velocity_linear->set_x(x);
  velocity_linear->set_y(y);
  velocity_linear->set_z(z);
  auto* velocity_angular = mutable_kinematic_state->mutable_velocity_of_body_in_vision()->mutable_angular();
  velocity_angular->set_x(rx);
  velocity_angular->set_y(ry);
  velocity_angular->set_z(rz);
}

inline void addAcquisitionTimestamp(::bosdyn::api::KinematicState* mutable_kinematic_state,
                                    const google::protobuf::Timestamp& timestamp) {
  mutable_kinematic_state->mutable_acquisition_timestamp()->CopyFrom(timestamp);
}
}  // namespace spot_ros2::test
