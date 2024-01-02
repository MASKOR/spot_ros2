// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/api/robot_state.pb.h>
#include <spot_driver_cpp/api/default_robot_state_client.hpp>
#include <spot_driver_cpp/api/time_sync_api.hpp>
#include <spot_driver_cpp/conversions/geometry.hpp>
#include <spot_driver_cpp/conversions/robot_state.hpp>

namespace spot_ros2 {

DefaultRobotStateClient::DefaultRobotStateClient(::bosdyn::client::RobotStateClient* client,
                                                 std::shared_ptr<TimeSyncApi> time_sync_api,
                                                 const std::string& robot_name)
    : client_{client}, time_sync_api_{time_sync_api}, frame_prefix_{robot_name.empty() ? "" : robot_name + "/"} {}

tl::expected<RobotState, std::string> DefaultRobotStateClient::getRobotState(const std::string& preferred_odom_frame) {
  std::shared_future<::bosdyn::client::RobotStateResultType> get_robot_state_result_future =
      client_->GetRobotStateAsync();

  ::bosdyn::client::RobotStateResultType get_robot_state_result = get_robot_state_result_future.get();
  if (!get_robot_state_result.status || !get_robot_state_result.response.has_robot_state()) {
    return tl::make_unexpected("Failed to get robot state: " + get_robot_state_result.status.DebugString());
  }

  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    return tl::make_unexpected("Failed to get latest clock skew: " + clock_skew_result.error());
  }

  const auto robot_state = get_robot_state_result.response.robot_state();

  const auto out = RobotState{
      GetBatteryStates(robot_state, clock_skew_result.value()),
      GetWifiState(robot_state),
      GetFootState(robot_state),
      GetEstopStates(robot_state, clock_skew_result.value()),
      GetJointStates(robot_state, clock_skew_result.value(), frame_prefix_),
      GetTf(robot_state, clock_skew_result.value(), frame_prefix_, preferred_odom_frame),
      GetOdomTwist(robot_state, clock_skew_result.value()),
      GetOdom(robot_state, clock_skew_result.value(), frame_prefix_, preferred_odom_frame == frame_prefix_ + "vision"),
      GetPowerState(robot_state, clock_skew_result.value()),
      GetSystemFaultState(robot_state, clock_skew_result.value()),
      GetManipulatorState(robot_state),
      GetEndEffectorForce(robot_state, clock_skew_result.value(), frame_prefix_),
      GetBehaviorFaultState(robot_state, clock_skew_result.value())};

  return out;
}

}  // namespace spot_ros2
