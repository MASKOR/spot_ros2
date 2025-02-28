// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/image_stitcher/image_stitcher_node.hpp>

#include <rclcpp/node.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_listener_interface.hpp>

namespace spot_ros2 {
ImageStitcherNode::ImageStitcherNode(const rclcpp::NodeOptions& options) {
  node_ = std::make_shared<rclcpp::Node>("image_stitcher", options);

  initialize(std::make_unique<RclcppCameraSynchronizer>(node_), std::make_unique<RclcppTfListenerInterface>(node_),
             std::make_unique<RclcppLoggerInterface>(node_->get_logger()),
             std::make_unique<RclcppParameterInterface>(node_));
}

void ImageStitcherNode::initialize(std::unique_ptr<CameraSynchronizerBase> synchronizer,
                                   std::unique_ptr<TfListenerInterfaceBase> tf_listener,
                                   std::unique_ptr<LoggerInterfaceBase> logger,
                                   std::unique_ptr<ParameterInterfaceBase> parameter_interface) {
  const std::string frame_prefix = parameter_interface->getFramePrefixWithDefaultFallback();

  stitcher_ = std::make_unique<ImageStitcher>(std::move(synchronizer), std::move(tf_listener),
                                              std::make_unique<RclcppCameraHandle>(node_, frame_prefix),
                                              std::move(logger), std::move(parameter_interface));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> ImageStitcherNode::get_node_base_interface() {
  return node_->get_node_base_interface();
}

}  // namespace spot_ros2
