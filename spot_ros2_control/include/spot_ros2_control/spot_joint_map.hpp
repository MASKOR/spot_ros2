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
#include "spot_hardware_interface/spot_constants.hpp"

#pragma once

namespace spot_ros2_control {

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
std::unordered_map<std::string, size_t> get_namespaced_joint_map(const std::string& spot_name, bool has_arm);

/// @brief Given a list of joints from a JointStates message, put them in the correct order that the Spot Hardware
/// interface expects.
/// @param spot_name Namespace that the ros2 control stack was launched in that prefixes the joint names
/// @param input_joint_states The JointStates message received from the robot
/// @param output_joint_states A JointStates message that will be ordered properly
/// @return boolean indicating if the joint angles got ordered successfully.
bool order_joint_states(const std::string& spot_name, const sensor_msgs::msg::JointState& input_joint_states,
                        sensor_msgs::msg::JointState& output_joint_states);

/// @brief Given a joint name (possibly with namespace), return the joint index
/// @param joint_str string name of joint
/// @param has_arm whether or not the spot has an arm (default true)
/// @return joint index
int get_joint_index(const std::string& joint_str, bool has_arm = true);

}  // namespace spot_ros2_control
