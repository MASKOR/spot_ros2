// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
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

#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "spot_hardware_interface/spot_constants.hpp"
#include "spot_hardware_interface/visibility_control.h"

#include "bosdyn/client/lease/lease_keepalive.h"
#include "bosdyn/client/robot_command/robot_command_builder.h"
#include "bosdyn/client/robot_command/robot_command_client.h"
#include "bosdyn/client/robot_command/robot_command_helpers.h"
#include "bosdyn/client/robot_command/robot_command_streaming_client.h"
#include "bosdyn/client/robot_state/robot_state_client.h"
#include "bosdyn/client/robot_state/robot_state_streaming_client.h"
#include "bosdyn/client/sdk/client_sdk.h"
#include "bosdyn/client/service_client/client_header_handling.h"
#include "bosdyn/client/time_sync/time_sync_helpers.h"
#include "bosdyn/client/util/cli_util.h"

using StateHandler = std::function<void(::bosdyn::api::RobotStateStreamResponse&)>;

namespace spot_hardware_interface {

struct JointStates {
  // This struct is used to hold a set of joint states of the robot.
  // The first 12 entries will be the leg joints in the following order:
  // FL hip x, FL hip y, FL knee, FR hip x, FR hip y, FR knee, RL hip x, RL hip y, RL knee, RR hip x, RR hip y, RR knee
  // And, if the robot has an arm, the 7 arm joints follow in this order:
  // sh0, sh1, el0, el1, wr0, wr1, f1x
  std::vector<float> position;  // in rad
  std::vector<float> velocity;  // in rad/s
  std::vector<float> load;      // in Nm
};

struct JointCommands {
  // This struct is used to hold a set of joint commands of the robot.
  // The first 12 entries will be the leg commands in the following order:
  // FL hip x, FL hip y, FL knee, FR hip x, FR hip y, FR knee, RL hip x, RL hip y, RL knee, RR hip x, RR hip y, RR knee
  // And, if the robot has an arm, the 7 arm commands follow in this order:
  // sh0, sh1, el0, el1, wr0, wr1, f1x
  std::vector<float> position;  // in rad
  std::vector<float> velocity;  // in rad/s
  std::vector<float> load;      // in Nm
  std::vector<float> k_q_p;
  std::vector<float> k_qd_p;
};

class StateStreamingHandler {
 public:
  /**
   * @brief Update member variables with the current position, velocity, and load of the robot's joints.
   * @param robot_state Robot state protobuf holding the current joint state of the robot.
   */
  void handle_state_streaming(::bosdyn::api::RobotStateStreamResponse& robot_state);
  /**
   * @brief Get a struct of the current joint states of the robot.
   * @return JointStates struct containing vectors of position, velocity, and load values.
   */
  void get_joint_states(JointStates& joint_states);
  /**
   * @brief Save the current foot states of the robot where:
   *  CONTACT_UNKNOWN	0	Unknown contact. Do not use.
      CONTACT_MADE	1	The foot is currently in contact with the ground.
      CONTACT_LOST	2	The foot is not in contact with the ground.
   */
  void get_foot_states(std::vector<int>& foot_states);

 private:
  // Stores the current position, velocity, and load of the robot's joints.
  std::vector<float> current_position_;
  std::vector<float> current_velocity_;
  std::vector<float> current_load_;
  // store the current foot contact states
  std::vector<int> current_foot_state_;
  // responsible for ensuring read/writes of joint states do not happen at the same time.
  std::mutex mutex_;
};

class SpotHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SpotHardware)

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SPOT_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // constants used for expanding the available command interfaces
  const std::string HW_IF_K_Q_P = "k_q_p";
  const std::string HW_IF_K_QD_P = "k_qd_p";
  // The 3 command interfaces are position, velocity, effort, k_q_p, and k_qd_p.
  static constexpr size_t command_interfaces_per_joint_ = 5;
  // The 3 state interfaces are position, velocity, and effort.
  static constexpr size_t state_interfaces_per_joint_ = 3;
  size_t njoints_;
  size_t nfeet = 4;

  // Login info
  std::string hostname_;
  std::string username_;
  std::string password_;

  // Stores gains to be used in the joint level command
  std::vector<float> k_q_p_;
  std::vector<float> k_qd_p_;

  // Power status
  bool powered_on_ = false;

  // Shared BD clients.
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  ::bosdyn::client::LeaseClient* lease_client_;
  ::bosdyn::client::RobotStateStreamingClient* state_client_;
  ::bosdyn::client::RobotCommandStreamingClient* command_stream_service_;
  ::bosdyn::client::RobotCommandClient* command_client_;

  // Holds joint states of the robot received from the BD SDK
  JointStates joint_states_;
  // Holds joint commands for the robot to send to BD SDK
  JointCommands joint_commands_;
  // Holds foot states received from the BD SDK
  std::vector<int> foot_states_;

  // Thread for reading the state of the robot.
  std::jthread state_thread_;
  // Simple class used in the state streaming thread that stores the current joint states of the robot.
  StateStreamingHandler state_streaming_handler_;
  bool state_stream_started_ = false;
  bool robot_authenticated_ = false;

  bool command_stream_started_ = false;
  bool init_state_ = false;

  ::bosdyn::client::TimeSyncEndpoint* endpoint_ = nullptr;

  ::bosdyn::api::JointControlStreamRequest joint_request_;

  // The following are functions that interact with the BD SDK to set up the robot and get the robot states.

  /**
   * @brief Create the ::bosdyn::client::Robot object and authenticate with the login information
   * @param hostname IP address of the robot
   * @param username Username for robot login
   * @param password Password for robot login
   * @return True if robot object is successfully created and authenticated, false otherwise.
   */
  bool authenticate_robot(const std::string& hostname, const std::string& username, const std::string& password);
  /**
   * @brief Start time sync threads with the ::bosdyn::client::Robot object
   * @return True if time sync successfully initialized and started, false otherwise.
   */
  bool start_time_sync();
  /**
   * @brief Check the estop status of the robot
   * @return True if robot is not e-stopped, false otherwise.
   */
  bool check_estop();
  /**
   * @brief Get the body lease of the robot
   * @return True if lease client successfully created and body lease is acquired, false otherwise.
   */
  bool get_lease();
  /**
   * @brief Power the robot on
   * @return True if successfully powered on, false otherwise.
   */
  bool power_on();
  /**
   * @brief Power the robot off
   * @return True if successfully powered off, false otherwise.
   */
  bool power_off();
  /**
   * @brief Start streaming the state of the robot, and attach a callback to it
   * @param state_policy a functor to call with new state updates
   * @return True if state stream thread successfully created, false otherwise.
   */
  bool start_state_stream(StateHandler&& state_policy);
  /**
   * @brief Stop streaming the state of the robot by shutting down the associated threads.
   */
  void stop_state_stream();
  /**
   * @brief Release the body lease of the robot.
   */
  void release_lease();
  /**
   * @brief Start streaming commands to the robot, and attach a callback to it
   * @return True if command stream clients were successfully created, false otherwise.
   */
  bool start_command_stream();
  /**
   * @brief Stop streaming commands to the robot.
   */
  void stop_command_stream();
  /**
   * @brief Send a joint command to the robot.
   * @param joint_commands contains position, velocity, and load
   */
  void send_command(const JointCommands& joint_commands);

  // Vectors for storing the commands and states for the robot.
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace spot_hardware_interface
