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
#include "std_msgs/msg/float64_multi_array.hpp"

#pragma once

namespace spot_ros2_control {

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

// TODO(tcappellari): Find a cleaner + better way to load and change these
// Gain values https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/constants.hpp
// NOTE: these should be different depending on number of joints the robot has (arm or none)
// Right now this is just temporary to see if we can get the commands accepted by robot
// This could be handled via a parameter in the future.
// kp and kd gains for a robot without an arm
static const std::vector<float> no_arm_kp = {624, 936, 286, 624, 936, 286, 624, 936, 286, 624, 936, 286};
static const std::vector<float> no_arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04};
// kp and kd gains for a robot with an arm
static const std::vector<float> arm_kp = {624, 936, 286,  624, 936, 286, 624, 936, 286, 624,
                                          936, 286, 1020, 255, 204, 102, 102, 102, 16.0};
static const std::vector<float> arm_kd = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20,
                                          5.20, 2.04, 10.2, 15.3, 10.2, 2.04, 2.04, 2.04, 0.32};
/// @brief Given a list of joints from a JointStates message, put them in the correct order that the Spot Hardware
/// interface expects.
/// @param msg JointStates message
/// @param ordered_joint_angles_ Joint positions from the joint state message following the correct order.
/// @return boolean indicating if the joint angles got ordered successfully.
bool order_joints(const sensor_msgs::msg::JointState& msg, std::vector<double>& ordered_joint_angles) {
  const auto njoints = msg.position.size();
  ordered_joint_angles.resize(njoints);
  // Different joint index maps for arm-full and arm-less
  switch (njoints) {
    // case without arm
    case kNjointsNoArm:
      for (size_t i = 0; i < njoints; ++i) {
        // get the joint name
        const auto& joint_name = msg.name.at(i);
        try {
          const auto joint_index = kJointNameToIndexWithoutArm.at(joint_name);
          ordered_joint_angles.at(joint_index) = msg.position.at(i);
        } catch (const std::out_of_range& e) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotJointMap"), "Invalid joint: " << joint_name);
          return false;
        }
      }
      return true;

    // case with arm
    case kNjointsArm:
      for (size_t i = 0; i < njoints; i++) {
        // get the joint name
        const auto joint_name = msg.name.at(i);
        try {
          const auto joint_index = kJointNameToIndexWithArm.at(joint_name);
          ordered_joint_angles.at(joint_index) = msg.position.at(i);
        } catch (const std::out_of_range& e) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotHardware"), "Invalid joint: " << joint_name);
          return false;
        }
      }
      return true;

    default:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotHardware"), "Invalid number of joints: " << njoints);
      return false;
  }
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
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Cannot find joint %s in joint map.", joint_name.c_str());
    return -1;
  }
  int joint_idx = has_arm ? kJointNameToIndexWithArm.at(joint_name) : kJointNameToIndexWithoutArm.at(joint_name);

  return joint_idx;
}

}  // namespace spot_ros2_control
