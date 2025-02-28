// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <memory>
#include <spot_driver/images/spot_image_publisher_node.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/images/images_middleware_handle.hpp>
#include <spot_driver/images/spot_image_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace {
constexpr auto kSDKClientName = "spot_image_publisher";
}

namespace spot_ros2::images {
SpotImagePublisherNode::SpotImagePublisherNode(std::unique_ptr<SpotApi> spot_api,
                                               std::unique_ptr<SpotImagePublisher::MiddlewareHandle> mw_handle,
                                               std::unique_ptr<ParameterInterfaceBase> parameters,
                                               std::unique_ptr<LoggerInterfaceBase> logger,
                                               std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                               std::unique_ptr<TimerInterfaceBase> timer,
                                               std::unique_ptr<NodeInterfaceBase> node_base_interface)
    : node_base_interface_{std::move(node_base_interface)} {
  initialize(std::move(spot_api), std::move(mw_handle), std::move(parameters), std::move(logger),
             std::move(tf_broadcaster), std::move(timer));
}

SpotImagePublisherNode::SpotImagePublisherNode(const rclcpp::NodeOptions& node_options) {
  const auto node = std::make_shared<rclcpp::Node>("image_publisher", node_options);
  node_base_interface_ = std::make_unique<RclcppNodeInterface>(node->get_node_base_interface());

  auto mw_handle = std::make_unique<ImagesMiddlewareHandle>(node);
  auto parameters = std::make_unique<RclcppParameterInterface>(node);
  auto logger = std::make_unique<RclcppLoggerInterface>(node->get_logger());
  auto tf_broadcaster = std::make_unique<RclcppTfBroadcasterInterface>(node);
  auto timer = std::make_unique<RclcppWallTimerInterface>(node);

  const auto timesync_timeout = parameters->getTimeSyncTimeout();
  auto spot_api = std::make_unique<DefaultSpotApi>(kSDKClientName, timesync_timeout, parameters->getCertificate());

  initialize(std::move(spot_api), std::move(mw_handle), std::move(parameters), std::move(logger),
             std::move(tf_broadcaster), std::move(timer));
}

void SpotImagePublisherNode::initialize(std::unique_ptr<SpotApi> spot_api,
                                        std::unique_ptr<SpotImagePublisher::MiddlewareHandle> mw_handle,
                                        std::unique_ptr<ParameterInterfaceBase> parameters,
                                        std::unique_ptr<LoggerInterfaceBase> logger,
                                        std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                        std::unique_ptr<TimerInterfaceBase> timer) {
  spot_api_ = std::move(spot_api);

  const auto hostname = parameters->getHostname();
  const auto port = parameters->getPort();
  const auto username = parameters->getUsername();
  const auto password = parameters->getPassword();
  const std::string frame_prefix = parameters->getFramePrefixWithDefaultFallback();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(hostname, port, frame_prefix); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
    const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  const auto expected_has_arm = spot_api_->hasArm();
  if (!expected_has_arm) {
    const auto error_msg{std::string{"Failed to check availability of spot arm: "}.append(expected_has_arm.error())};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ = std::make_unique<SpotImagePublisher>(spot_api_->image_client_interface(), std::move(mw_handle),
                                                   std::move(parameters), std::move(logger), std::move(tf_broadcaster),
                                                   std::move(timer), expected_has_arm.value());

  // TODO(jschornak): initialize() always returns true -- revise implementation to make it return void
  if (!internal_->initialize()) {
    constexpr auto error_msg{"Failed to initialize image publisher."};
    logger->logError(error_msg);
    throw std::runtime_error(error_msg);
  }
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotImagePublisherNode::get_node_base_interface() {
  return node_base_interface_->getNodeBaseInterface();
}

}  // namespace spot_ros2::images
