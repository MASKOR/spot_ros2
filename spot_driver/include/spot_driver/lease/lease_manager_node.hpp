// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/lease/lease_manager.hpp>

#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>

#include <memory>

namespace spot_ros2::lease {
class LeaseManagerNode {
 public:
  explicit LeaseManagerNode(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                            std::unique_ptr<LeaseManager::MiddlewareHandle> middleware_handle,
                            std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                            std::unique_ptr<TimerInterfaceBase> timer_interface,
                            std::unique_ptr<LoggerInterfaceBase> logger_interface);

  explicit LeaseManagerNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container without requiring that it inherit
   * from rclcpp::Node.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<LeaseManager> internal_;

  void initialize(std::shared_ptr<rclcpp::Node> node, std::unique_ptr<SpotApi> spot_api,
                  std::unique_ptr<LeaseManager::MiddlewareHandle> middleware_handle,
                  std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                  std::unique_ptr<TimerInterfaceBase> timer_interface,
                  std::unique_ptr<LoggerInterfaceBase> logger_interface);
};
}  // namespace spot_ros2::lease
