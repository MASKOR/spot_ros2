// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 PAL Robotics S.L.
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

#include "spot_controllers/forward_state_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace spot_controllers {
ForwardStateController::ForwardStateController() : forward_command_controller::ForwardControllersBase() {}

void ForwardStateController::declare_parameters() {
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ForwardStateController::read_parameters() {
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Example: if you input joints [1,2,3] and interfaces [A,B,C] as parameters, the order of the command will be
  // [1/A, 1/B, 1/C, 2/A, 2/B, 2/C, 3/A, 3/B, 3/C]
  for (const auto& interface_name : params_.interface_names) {
    for (const auto& joint : params_.joints) {
      command_interface_types_.push_back(joint + "/" + interface_name);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace spot_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_controllers::ForwardStateController, controller_interface::ControllerInterface)
