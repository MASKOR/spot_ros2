// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <spot_driver/image_stitcher/image_stitcher.hpp>

namespace spot_ros2 {
class ImageStitcherNode {
 public:
  explicit ImageStitcherNode(const rclcpp::NodeOptions& options);

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  ImageStitcher stitcher_;
};
}  // namespace spot_ros2
