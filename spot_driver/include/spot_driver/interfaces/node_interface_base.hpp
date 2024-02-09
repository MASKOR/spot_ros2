// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that logs messages at different severity levels.
 */
class NodeInterfaceBase {
 public:
  virtual ~NodeInterfaceBase() = default;

  virtual std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface() = 0;
};
}  // namespace spot_ros2
