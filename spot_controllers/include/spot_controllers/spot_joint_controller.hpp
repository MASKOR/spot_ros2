// File modified. Modifications Copyright (c) 2025 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <spot_controllers/spot_joint_controller_parameters.hpp>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "spot_controllers/visibility_control.h"
#include "spot_msgs/msg/joint_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace spot_controllers {
using CmdType = spot_msgs::msg::JointCommand;

/**
 * \brief Controller for Spot capable of streaming any combination of position, velocity, load, k_q_p, and k_qd_p
 * to the Spot hardware interface for a given set of joints.
 *
 * This controller parses the information provided in the JointCommand message received to determine which command
 * interfaces to control.
 *
 * Subscribes to:
 * - \b joint_commands (spot_msgs::msg::JointCommand) : The commands to apply. The `name` field should contain the
 * names of the joints to command (this can be any number of joints, from one to all of the joints on the robot).
 * For each joint name entry, choose which interface/s you want to control by filling out the correspoinding index in
 * the other fields of the JointCommand message (position, velocity, effort, k_q_p, and/or k_q_d)
 */
class SpotJointController : public controller_interface::ControllerInterface {
 public:
  SpotJointController();

  ~SpotJointController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  using Params = spot_joint_controller::Params;
  using ParamListener = spot_joint_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  // Constants useful for interface setup / sending commands.
  std::vector<std::string> interfaces_ = {"position", "velocity", "effort", "k_q_p", "k_qd_p"};
  static constexpr size_t n_interfaces_ = 5;
  static constexpr size_t position_offset_ = 0;
  static constexpr size_t velocity_offset_ = 1;
  static constexpr size_t effort_offset_ = 2;
  static constexpr size_t k_q_p_offset_ = 3;
  static constexpr size_t k_qd_p_offset_ = 4;
};

}  // namespace spot_controllers
