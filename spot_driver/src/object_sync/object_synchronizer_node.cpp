// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/object_sync/object_synchronizer_node.hpp>

#include <memory>
#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/interfaces/rclcpp_clock_interface.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_listener_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/object_sync/object_synchronizer.hpp>
#include <spot_driver/robot_state/state_middleware_handle.hpp>

namespace {
constexpr auto kDefaultSDKName{"object_sync"};
}

namespace spot_ros2 {

ObjectSynchronizerNode::ObjectSynchronizerNode(std::unique_ptr<NodeInterfaceBase> node_base_interface,
                                               std::unique_ptr<SpotApi> spot_api,
                                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                               std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                                               std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                                               std::unique_ptr<TimerInterfaceBase> world_object_update_timer,
                                               std::unique_ptr<TimerInterfaceBase> tf_broadcaster_timer,
                                               std::unique_ptr<ClockInterfaceBase> clock_interface)
    : node_base_interface_{std::move(node_base_interface)} {
  initialize(std::move(spot_api), std::move(parameter_interface), std::move(logger_interface),
             std::move(tf_broadcaster_interface), std::move(tf_listener_interface),
             std::move(world_object_update_timer), std::move(tf_broadcaster_timer), std::move(clock_interface));
}

ObjectSynchronizerNode::ObjectSynchronizerNode(const rclcpp::NodeOptions& node_options) {
  const auto node = std::make_shared<rclcpp::Node>("object_sync", node_options);
  node_base_interface_ = std::make_unique<RclcppNodeInterface>(node->get_node_base_interface());

  auto mw_handle = std::make_unique<StateMiddlewareHandle>(node);
  auto parameter_interface = std::make_unique<RclcppParameterInterface>(node);
  auto logger_interface = std::make_unique<RclcppLoggerInterface>(node->get_logger());
  auto tf_broadcaster_interface = std::make_unique<RclcppTfBroadcasterInterface>(node);
  auto tf_listener_interface = std::make_unique<RclcppTfListenerInterface>(node);
  auto world_object_update_timer = std::make_unique<RclcppWallTimerInterface>(node);
  auto tf_broadcaster_timer = std::make_unique<RclcppWallTimerInterface>(node);
  auto clock_interface = std::make_unique<RclcppClockInterface>(node->get_node_clock_interface());

  const auto timesync_timeout = parameter_interface->getTimeSyncTimeout();
  auto spot_api =
      std::make_unique<DefaultSpotApi>(kDefaultSDKName, timesync_timeout, parameter_interface->getCertificate());

  initialize(std::move(spot_api), std::move(parameter_interface), std::move(logger_interface),
             std::move(tf_broadcaster_interface), std::move(tf_listener_interface),
             std::move(world_object_update_timer), std::move(tf_broadcaster_timer), std::move(clock_interface));
}

void ObjectSynchronizerNode::initialize(std::unique_ptr<SpotApi> spot_api,
                                        std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                                        std::unique_ptr<LoggerInterfaceBase> logger_interface,
                                        std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_interface,
                                        std::unique_ptr<TfListenerInterfaceBase> tf_listener_interface,
                                        std::unique_ptr<TimerInterfaceBase> world_object_update_timer,
                                        std::unique_ptr<TimerInterfaceBase> tf_broadcaster_timer,
                                        std::unique_ptr<ClockInterfaceBase> clock_interface) {
  spot_api_ = std::move(spot_api);

  const auto address = parameter_interface->getHostname();
  const auto port = parameter_interface->getPort();
  const auto username = parameter_interface->getUsername();
  const auto password = parameter_interface->getPassword();
  const std::string frame_prefix = parameter_interface->getFramePrefixWithDefaultFallback();

  // create and authenticate robot
  if (const auto create_robot_result = spot_api_->createRobot(address, port, frame_prefix); !create_robot_result) {
    const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
    const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
    logger_interface->logError(error_msg);
    throw std::runtime_error(error_msg);
  }

  internal_ = std::make_unique<ObjectSynchronizer>(
      spot_api_->worldObjectClientInterface(), spot_api_->timeSyncInterface(), std::move(parameter_interface),
      std::move(logger_interface), std::move(tf_broadcaster_interface), std::move(tf_listener_interface),
      std::move(world_object_update_timer), std::move(tf_broadcaster_timer), std::move(clock_interface));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> ObjectSynchronizerNode::get_node_base_interface() {
  return node_base_interface_->getNodeBaseInterface();
}

}  // namespace spot_ros2
