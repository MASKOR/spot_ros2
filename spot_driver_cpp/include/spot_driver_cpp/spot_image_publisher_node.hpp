// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver_cpp/api/spot_api.hpp>
#include <spot_driver_cpp/interfaces/middleware_interface_base.hpp>
#include <spot_driver_cpp/images/spot_image_publisher.hpp>

#include <memory>

namespace spot_ros2 {
/**
 * @brief Wraps SpotImagePublisher to allow using it like a rclcpp::Node.
 */
class SpotImagePublisherNode {
 public:
  SpotImagePublisherNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MiddlewareInterface> middleware_interface,
                         std::unique_ptr<SpotApi> spot_api);
  explicit SpotImagePublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

  /**
   * @brief Returns the NodeBaseInterface of this class's node.
   * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
   * This allows loading this class as a component node in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
   */
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MiddlewareInterface> middleware_interface_;
  std::unique_ptr<SpotApi> spot_api_;
  std::unique_ptr<images::SpotImagePublisher> internal_;
};
}  // namespace spot_ros2
