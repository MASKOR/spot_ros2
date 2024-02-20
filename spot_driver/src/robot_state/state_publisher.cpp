// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <chrono>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/robot_state.hpp>
#include <spot_driver/interfaces/rclcpp_publisher_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/robot_state/state_publisher.hpp>
#include <spot_driver/types.hpp>
#include <utility>

namespace {
constexpr auto kRobotStateCallbackPeriod = std::chrono::duration<double>{1.0 / 50.0};  // 50 Hz
}

namespace spot_ros2 {

StatePublisher::StatePublisher(const std::shared_ptr<StateClientInterface>& state_client_interface,
                               const std::shared_ptr<TimeSyncApi>& time_sync_api,
                               std::unique_ptr<MiddlewareHandle> middleware_handle,
                               std::unique_ptr<PublisherInterfaceBase<spot_msgs::msg::WiFiState>> wi_fi_state_publisher,
                               std::unique_ptr<ParameterInterfaceBase> parameter_interface,
                               std::unique_ptr<LoggerInterfaceBase> logger_interface,
                               std::unique_ptr<TfInterfaceBase> tf_interface,
                               std::unique_ptr<TimerInterfaceBase> timer_interface)
    : is_using_vision_{false},
      state_client_interface_{state_client_interface},
      time_sync_interface_{time_sync_api},
      middleware_handle_{std::move(middleware_handle)},
      wi_fi_state_publisher_{std::move(wi_fi_state_publisher)},
      parameter_interface_{std::move(parameter_interface)},
      logger_interface_{std::move(logger_interface)},
      tf_interface_{std::move(tf_interface)},
      timer_interface_{std::move(timer_interface)} {
  const auto spot_name = parameter_interface_->getSpotName();
  frame_prefix_ = spot_name.empty() ? "" : spot_name + "/";

  const auto preferred_odom_frame = parameter_interface_->getPreferredOdomFrame();
  is_using_vision_ = preferred_odom_frame == "vision";
  full_odom_frame_id_ = preferred_odom_frame.find('/') == std::string::npos ? spot_name + "/" + preferred_odom_frame
                                                                            : preferred_odom_frame;

  // Create a timer to request and publish robot state at a fixed rate
  timer_interface_->setTimer(kRobotStateCallbackPeriod, [this] {
    timerCallback();
  });
}

void StatePublisher::timerCallback() {
  // Get latest clock skew each time we request a robot state
  const auto clock_skew_result = time_sync_interface_->getClockSkew();
  if (!clock_skew_result) {
    logger_interface_->logError(std::string{"Failed to get latest clock skew: "}.append(clock_skew_result.error()));
    return;
  }

  const auto robot_state_result = state_client_interface_->getRobotState();
  if (!robot_state_result.has_value()) {
    logger_interface_->logError(std::string{"Failed to get robot_state: "}.append(robot_state_result.error()));
    return;
  }

  const auto& clock_skew = clock_skew_result.value();
  const auto& robot_state = robot_state_result.value();

  const auto robot_state_messages =
      RobotStateMessages{getBatteryStates(*robot_state_result, clock_skew),
                         getWifiState(robot_state),
                         getFootState(robot_state),
                         getEstopStates(robot_state, clock_skew),
                         getJointStates(robot_state, clock_skew, frame_prefix_),
                         getTf(robot_state, clock_skew, frame_prefix_, full_odom_frame_id_),
                         getOdomTwist(robot_state, clock_skew),
                         getOdom(robot_state, clock_skew, frame_prefix_, is_using_vision_),
                         getPowerState(robot_state, clock_skew),
                         getSystemFaultState(robot_state, clock_skew),
                         getManipulatorState(robot_state),
                         getEndEffectorForce(robot_state, clock_skew, frame_prefix_),
                         getBehaviorFaultState(robot_state, clock_skew)};

  middleware_handle_->publishRobotState(robot_state_messages);

  wi_fi_state_publisher_->publish(getWifiState(robot_state));

  if (robot_state_messages.maybe_tf) {
    tf_interface_->sendDynamicTransforms(robot_state_messages.maybe_tf->transforms);
  }
}

}  // namespace spot_ros2
