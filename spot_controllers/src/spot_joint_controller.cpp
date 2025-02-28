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

#include "spot_controllers/spot_joint_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace spot_controllers {
SpotJointController::SpotJointController()
    : controller_interface::ControllerInterface(), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr) {}

controller_interface::CallbackReturn SpotJointController::on_init() {
  auto param_listener = std::make_shared<ParamListener>(get_node());
  params_ = param_listener->get_params();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotJointController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>("~/joint_commands", rclcpp::SystemDefaultsQoS(),
                                                                        [this](const CmdType::SharedPtr msg) {
                                                                          rt_command_ptr_.writeFromNonRT(msg);
                                                                        });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpotJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : params_.joints) {
    // this sets the interfaces that get claimed.
    for (const auto& interface : interfaces_) {
      command_interfaces_config.names.push_back(joint + "/" + interface);
    }
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SpotJointController::state_interface_configuration() const {
  // this controller has no knowledge of robot state... but it could (i.e. basic command validation?)
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn SpotJointController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, command_interface_types_, std::string(""),
                                                    ordered_interfaces) ||
      command_interface_types_.size() != ordered_interfaces.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu", command_interface_types_.size(),
                 ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpotJointController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpotJointController::update(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands)) {
    return controller_interface::return_type::OK;
  }

  const auto& joint_names = (*joint_commands)->name;
  const auto& njoints_to_command = joint_names.size();

  const bool using_position = (*joint_commands)->position.size() == njoints_to_command;
  const bool using_velocity = (*joint_commands)->velocity.size() == njoints_to_command;
  const bool using_effort = (*joint_commands)->effort.size() == njoints_to_command;
  const bool using_k_q_p = (*joint_commands)->k_q_p.size() == njoints_to_command;
  const bool using_k_qd_p = (*joint_commands)->k_qd_p.size() == njoints_to_command;

  // Iterate through the names of the joints that we want to command.
  for (size_t i = 0; i < njoints_to_command; i++) {
    const auto joint_name = joint_names.at(i);
    // find the command index
    const auto& it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
    if (it == params_.joints.end()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                           "Joint %s was not in the command interface!", joint_name.c_str());
      continue;
    }
    // get the index that the name is at in the command
    const auto command_index = std::distance(params_.joints.begin(), it);
    if (using_position) {
      command_interfaces_.at(n_interfaces_ * command_index + position_offset_)
          .set_value((*joint_commands)->position.at(i));
    }
    if (using_velocity) {
      command_interfaces_.at(n_interfaces_ * command_index + velocity_offset_)
          .set_value((*joint_commands)->velocity.at(i));
    }
    if (using_effort) {
      command_interfaces_.at(n_interfaces_ * command_index + effort_offset_).set_value((*joint_commands)->effort.at(i));
    }
    if (using_k_q_p) {
      command_interfaces_.at(n_interfaces_ * command_index + k_q_p_offset_).set_value((*joint_commands)->k_q_p.at(i));
    }
    if (using_k_qd_p) {
      command_interfaces_.at(n_interfaces_ * command_index + k_qd_p_offset_).set_value((*joint_commands)->k_qd_p.at(i));
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::SpotJointController, controller_interface::ControllerInterface)
