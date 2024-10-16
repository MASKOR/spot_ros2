// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#pragma once

namespace spot_ros2_control {

// Gain values https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/constants.hpp
// This will be handled via a parameter in the future so there is the option to change them, for now they are hardcoded

// kp and kd gains for a robot without an arm
const std::vector<float> no_arm_kp = {624, 936, 286, 624, 936, 286, 624, 936, 286, 624, 936, 286};
const std::vector<float> no_arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04};
// kp and kd gains for a robot with an arm
const std::vector<float> arm_kp = {624, 936, 286,  624, 936, 286, 624, 936, 286, 624,
                                   936, 286, 1020, 255, 204, 102, 102, 102, 16.0};
const std::vector<float> arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20,
                                   5.20, 2.04, 10.2, 15.3, 10.2, 2.04, 2.04, 2.04, 0.32};

/// @brief Number of joints we expect if the robot has an arm
inline constexpr int kNjointsArm = 19;
/// @brief Number of joints we expect if the robot has no arm
inline constexpr int kNjointsNoArm = 12;

/// @brief Maps joint name to desired joint index for robots with arms
static const std::unordered_map<std::string, size_t> kJointNameToIndexWithArm{
    {"front_left_hip_x", 0},  {"front_left_hip_y", 1}, {"front_left_knee", 2},   {"front_right_hip_x", 3},
    {"front_right_hip_y", 4}, {"front_right_knee", 5}, {"rear_left_hip_x", 6},   {"rear_left_hip_y", 7},
    {"rear_left_knee", 8},    {"rear_right_hip_x", 9}, {"rear_right_hip_y", 10}, {"rear_right_knee", 11},
    {"arm_sh0", 12},          {"arm_sh1", 13},         {"arm_el0", 14},          {"arm_el1", 15},
    {"arm_wr0", 16},          {"arm_wr1", 17},         {"arm_f1x", 18},
};
/// @brief Maps joint name to joint index for robots without arms.
static const std::unordered_map<std::string, size_t> kJointNameToIndexWithoutArm{
    {"front_left_hip_x", 0},  {"front_left_hip_y", 1}, {"front_left_knee", 2},   {"front_right_hip_x", 3},
    {"front_right_hip_y", 4}, {"front_right_knee", 5}, {"rear_left_hip_x", 6},   {"rear_left_hip_y", 7},
    {"rear_left_knee", 8},    {"rear_right_hip_x", 9}, {"rear_right_hip_y", 10}, {"rear_right_knee", 11},
};

/// @brief Return the joint name to index map depending on the namespace and if the robot has an arm.
/// @param spot_name Namespace that the ros2 control stack was launched in that prefixes the joint names
/// @param has_arm Boolean indicating if the arm joint angles should be included in the map
/// @return Unordered map that takes joint name to joint index.
std::unordered_map<std::string, size_t> get_namespaced_joint_map(const std::string& spot_name, bool has_arm) {
  const auto default_map = has_arm ? kJointNameToIndexWithArm : kJointNameToIndexWithoutArm;
  if (spot_name.empty()) {
    return default_map;
  }
  std::unordered_map<std::string, size_t> namespaced_map;
  const std::string joint_prefix = spot_name + "/";
  for (const auto& pair : default_map) {
    namespaced_map[joint_prefix + pair.first] = pair.second;
  }
  return namespaced_map;
}

/// @brief Given a list of joints from a JointStates message, put them in the correct order that the Spot Hardware
/// interface expects.
/// @param spot_name Namespace that the ros2 control stack was launched in that prefixes the joint names
/// @param input_joint_states The JointStates message received from the robot
/// @param output_joint_states A JointStates message that will be ordered properly
/// @return boolean indicating if the joint angles got ordered successfully.
bool order_joint_states(const std::string& spot_name, const sensor_msgs::msg::JointState& input_joint_states,
                        sensor_msgs::msg::JointState& output_joint_states) {
  const auto njoints = input_joint_states.position.size();
  bool has_arm;
  if (njoints == kNjointsArm) {
    has_arm = true;
  } else if (njoints == kNjointsNoArm) {
    has_arm = false;
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotJointMap"), "Invalid number of joints: " << njoints);
    return false;
  }

  output_joint_states.name.resize(njoints);
  output_joint_states.position.resize(njoints);
  output_joint_states.velocity.resize(njoints);
  output_joint_states.effort.resize(njoints);

  const auto joint_map = get_namespaced_joint_map(spot_name, has_arm);

  for (size_t i = 0; i < njoints; ++i) {
    // get the joint name
    const auto& joint_name = input_joint_states.name.at(i);
    try {
      const auto joint_index = joint_map.at(joint_name);
      output_joint_states.name.at(joint_index) = joint_name;
      output_joint_states.position.at(joint_index) = input_joint_states.position.at(i);
      output_joint_states.velocity.at(joint_index) = input_joint_states.velocity.at(i);
      output_joint_states.effort.at(joint_index) = input_joint_states.effort.at(i);
    } catch (const std::out_of_range& e) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotJointMap"), "Invalid joint: " << joint_name);
      return false;
    }
  }
  return true;
}

/// @brief Given a joint name (possibly with namespace), return the joint index
/// @param joint_str string name of joint
/// @param has_arm whether or not the spot has an arm (default true)
/// @return joint index
int get_joint_index(const std::string& joint_str, bool has_arm = true) {
  // Check if the joint_str has a namespace - if so, remove it
  size_t namespace_pos = joint_str.find("/");
  std::string joint_name = (namespace_pos != std::string::npos) ? joint_str.substr(namespace_pos + 1) : joint_str;

  if (kJointNameToIndexWithArm.find(joint_name) == kJointNameToIndexWithArm.end() &&
      kJointNameToIndexWithoutArm.find(joint_name) == kJointNameToIndexWithoutArm.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotJointMap"), "Cannot find joint %s in joint map.", joint_name.c_str());
    return -1;
  }
  int joint_idx = has_arm ? kJointNameToIndexWithArm.at(joint_name) : kJointNameToIndexWithoutArm.at(joint_name);

  return joint_idx;
}

}  // namespace spot_ros2_control
