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

#include "spot_hardware_interface/spot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "google/protobuf/util/time_util.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace spot_hardware_interface {

void StateStreamingHandler::handle_state_streaming(::bosdyn::api::RobotStateStreamResponse& robot_state) {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  // Get joint states from the robot and write them to the joint_states_ struct
  const auto& position_msg = robot_state.joint_states().position();
  const auto& velocity_msg = robot_state.joint_states().velocity();
  const auto& load_msg = robot_state.joint_states().load();
  current_position_.assign(position_msg.begin(), position_msg.end());
  current_velocity_.assign(velocity_msg.begin(), velocity_msg.end());
  current_load_.assign(load_msg.begin(), load_msg.end());
  // Save current foot contact states
  for (size_t i = 0; i < 4; i++) {
    current_foot_state_.push_back(robot_state.contact_states(i));
  }
}

void StateStreamingHandler::get_joint_states(JointStates& joint_states) {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  // Fill in members of the joint states stuct passed in by reference.
  joint_states.position.assign(current_position_.begin(), current_position_.end());
  joint_states.velocity.assign(current_velocity_.begin(), current_velocity_.end());
  joint_states.load.assign(current_load_.begin(), current_load_.end());
}

void StateStreamingHandler::get_foot_states(std::vector<int>& foot_states) {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  foot_states.assign(current_foot_state_.begin(), current_foot_state_.end());
}

hardware_interface::CallbackReturn SpotHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Get and save parameters
  hostname_ = info_.hardware_parameters["hostname"];
  username_ = info_.hardware_parameters["username"];
  password_ = info_.hardware_parameters["password"];

  hw_states_.resize(info_.joints.size() * state_interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size() * command_interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());

  njoints_ = hw_states_.size() / state_interfaces_per_joint_;

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // First check command interfaces
    if (joint.command_interfaces.size() != command_interfaces_per_joint_) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 1. check position
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 2. check velocity
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 3. check effort
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 4. check k_q_p
    if (joint.command_interfaces[3].name != HW_IF_K_Q_P) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), HW_IF_K_Q_P.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 5. check k_qd_p
    if (joint.command_interfaces[4].name != HW_IF_K_QD_P) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), HW_IF_K_QD_P.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Second, check state interfaces
    if (joint.state_interfaces.size() != state_interfaces_per_joint_) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 1. check position
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 2. check velocity
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 3. check effort
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset values always when configuring hardware
  hw_states_.assign(hw_states_.size(), 0);
  hw_commands_.assign(hw_commands_.size(), 0);
  // allocate the sizes of these vectors ahead of time
  joint_states_.position.assign(njoints_, 0);
  joint_states_.velocity.assign(njoints_, 0);
  joint_states_.load.assign(njoints_, 0);
  foot_states_.assign(nfeet, 0);

  // Set up the robot using the BD SDK and start command streaming.
  if (!authenticate_robot(hostname_, username_, password_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!start_time_sync()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!start_state_stream(std::bind(&StateStreamingHandler::handle_state_streaming, &state_streaming_handler_,
                                    std::placeholders::_1))) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!check_estop()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!get_lease()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!power_on()) {
    release_lease();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command streaming
  if (!start_command_stream()) {
    release_lease();
    power_off();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set up command_states struct, initialized to zeros
  joint_commands_.position = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.velocity = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.load = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.k_q_p = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.k_qd_p = std::vector<float>(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpotHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints.at(i);
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION,
                                                                     &hw_states_[state_interfaces_per_joint_ * i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY,
                                                                     &hw_states_[state_interfaces_per_joint_ * i + 1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT,
                                                                     &hw_states_[state_interfaces_per_joint_ * i + 2]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpotHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints.at(i);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[command_interfaces_per_joint_ * i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[command_interfaces_per_joint_ * i + 1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &hw_commands_[command_interfaces_per_joint_ * i + 2]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_K_Q_P, &hw_commands_[command_interfaces_per_joint_ * i + 3]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_K_QD_P, &hw_commands_[command_interfaces_per_joint_ * i + 4]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SpotHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_command_stream();
  release_lease();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_state_stream();
  stop_command_stream();
  release_lease();
  if (!power_off()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_state_stream();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpotHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  state_streaming_handler_.get_joint_states(joint_states_);
  const auto& joint_pos = joint_states_.position;
  const auto& joint_vel = joint_states_.velocity;
  const auto& joint_load = joint_states_.load;
  // wait for them to be initialized
  if (joint_pos.empty() || joint_vel.empty() || joint_load.empty()) {
    return hardware_interface::return_type::OK;
  }
  // Ensure that the states received from the Spot SDK will fit into the hw_states_ vector
  const auto states_size = hw_states_.size();
  if (state_interfaces_per_joint_ * joint_pos.size() != states_size ||
      state_interfaces_per_joint_ * joint_vel.size() != states_size ||
      state_interfaces_per_joint_ * joint_load.size() != states_size) {
    RCLCPP_FATAL(
        rclcpp::get_logger("SpotHardware"),
        "The number of joints and interfaces does not match with the outputted joint states from the Spot SDK!");
    return hardware_interface::return_type::ERROR;
  }
  // Read values into joint states
  for (size_t i = 0; i < joint_pos.size(); ++i) {
    hw_states_.at(i * state_interfaces_per_joint_) = joint_pos.at(i);
    hw_states_.at(i * state_interfaces_per_joint_ + 1) = joint_vel.at(i);
    hw_states_.at(i * state_interfaces_per_joint_ + 2) = joint_load.at(i);
  }

  // Fill in the initial command values
  if (!init_state_) {
    for (size_t i = 0; i < njoints_; i++) {
      // copy over current position, velocity, and load from state to current command
      hw_commands_[command_interfaces_per_joint_ * i] = hw_states_[state_interfaces_per_joint_ * i];
      hw_commands_[command_interfaces_per_joint_ * i + 1] = hw_states_[state_interfaces_per_joint_ * i + 1];
      hw_commands_[command_interfaces_per_joint_ * i + 2] = hw_states_[state_interfaces_per_joint_ * i + 2];
      // Fill in k_q_p and k_qd_p gains from the initial_value field from the URDF
      const auto& joint = info_.joints.at(i);
      hw_commands_[command_interfaces_per_joint_ * i + 3] = std::stof(joint.command_interfaces.at(3).initial_value);
      hw_commands_[command_interfaces_per_joint_ * i + 4] = std::stof(joint.command_interfaces.at(4).initial_value);
    }
    init_state_ = true;
  }

  state_streaming_handler_.get_foot_states(foot_states_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SpotHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // This function will be responsible for sending commands to the robot via the BD SDK -- currently unimplemented.
  if (!command_stream_started_) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Command streaming was not started");
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    joint_commands_.position.at(i) = hw_commands_[command_interfaces_per_joint_ * i];
    joint_commands_.velocity.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 1];
    joint_commands_.load.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 2];
    joint_commands_.k_q_p.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 3];
    joint_commands_.k_qd_p.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 4];
  }
  send_command(joint_commands_);

  return hardware_interface::return_type::OK;
}

bool SpotHardware::authenticate_robot(const std::string& hostname, const std::string& username,
                                      const std::string& password) {
  if (robot_authenticated_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot already authenticated!");
    return true;
  }
  // Create a Client SDK object.
  const auto client_sdk = ::bosdyn::client::CreateStandardSDK("SpotHardware");
  auto robot_result = client_sdk->CreateRobot(hostname);
  if (!robot_result) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot");
    return false;
  }
  robot_ = robot_result.move();
  const ::bosdyn::common::Status status = robot_->Authenticate(username, password);
  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"),
                 "Could not authenticate robot with hostname: %s username: %s password: %s", hostname.c_str(),
                 username.c_str(), password.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot successfully authenticated!");
  robot_authenticated_ = true;
  return true;
}

bool SpotHardware::start_time_sync() {
  auto start_response = robot_->StartTimeSync();
  auto time_sync_thread_resp = robot_->GetTimeSyncThread();
  if (!time_sync_thread_resp.status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not get time sync thread from robot");
    return false;
  }
  auto time_sync_thread = time_sync_thread_resp.response;
  if (!time_sync_thread->WaitForSync(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to establish time sync before timing out");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Time sync complete");
  return true;
}

bool SpotHardware::check_estop() {
  // Verify the robot is not estopped and that an external application has registered and holds
  // an estop endpoint.
  const auto estop_status = robot_->IsEstopped();
  if (!estop_status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not check estop status");
    return false;
  }
  if (estop_status.response) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Robot is e-stopped, cannot continue.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Estop check complete!");
  return true;
}

bool SpotHardware::get_lease() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Getting Lease");
  // First create a lease client.
  ::bosdyn::client::Result<::bosdyn::client::LeaseClient*> lease_client_resp =
      robot_->EnsureServiceClient<::bosdyn::client::LeaseClient>();
  if (!lease_client_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create lease client");
    return false;
  }
  lease_client_ = lease_client_resp.response;
  // Then acquire the lease for the body.
  const auto lease_res = lease_client_->TakeLease("body");
  if (!lease_res) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not acquire body lease");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Lease acquired!!");
  return true;
}

bool SpotHardware::power_on() {
  if (powered_on_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot is already powered on.");
    return true;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powering on...");
  const auto power_status = robot_->PowerOnMotors(std::chrono::seconds(60), 1.0);
  if (!power_status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not power on the robot");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powered on!");
  powered_on_ = true;
  return true;
}

bool SpotHardware::power_off() {
  if (!powered_on_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot is already powered off.");
    return true;
  }
  bosdyn::api::RobotCommand poweroff_command = ::bosdyn::client::SafePowerOffCommand();
  auto poweroff_res = command_client_->RobotCommand(poweroff_command);
  if (!poweroff_res) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to complete the safe power off command");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powered off!");
  powered_on_ = false;
  return true;
}

void state_stream_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateStreamingClient* stateStreamClient,
                       StateHandler&& state_policy) {
  ::bosdyn::api::RobotStateStreamResponse latest_state_stream_response;

  while (!stop_token.stop_requested()) {
    // Get robot state stream
    auto robot_state_stream = stateStreamClient->GetRobotStateStream();
    if (!robot_state_stream) {
      RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"),
                   "Failed to get robot state. Does the robot have a valid joint level control license?");
      continue;
    }
    latest_state_stream_response = std::move(robot_state_stream.response);
    state_policy(latest_state_stream_response);
  }
}

bool SpotHardware::start_state_stream(StateHandler&& state_policy) {
  if (state_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State stream has already been started!");
    return true;
  }
  // Start state streaming
  auto robot_state_stream_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotStateStreamingClient>();
  if (!robot_state_stream_client_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot state client");
    return false;
  }
  state_client_ = robot_state_stream_client_resp.move();
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot State Client created");

  state_thread_ = std::jthread(&spot_hardware_interface::state_stream_loop, state_client_, state_policy);
  state_stream_started_ = true;
  return true;
}

void SpotHardware::stop_state_stream() {
  if (!state_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State stream already stopped");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Stopping State Stream");
  state_thread_.request_stop();
  state_thread_.join();
  state_stream_started_ = false;
}

bool SpotHardware::start_command_stream() {
  if (command_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Command stream has already been started!");
    return true;
  }
  // Start command streaming
  auto robot_command_stream_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotCommandClient>();
  if (!robot_command_stream_client_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot command client");
    return false;
  }
  command_client_ = robot_command_stream_client_resp.response;

  auto endpoint_result = robot_->StartTimeSyncAndGetEndpoint();
  if (!endpoint_result) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not get timesync endpoint");
    return false;
  }

  command_client_->AddTimeSyncEndpoint(endpoint_result.response);

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot Command Client successfully created!");

  bosdyn::api::RobotCommand joint_command = ::bosdyn::client::JointCommand();
  auto joint_res = command_client_->RobotCommand(joint_command);
  if (!joint_res.status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to activate joint control mode");
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Message: %s", joint_res.status.DebugString().c_str());
    return false;
  }

  auto robot_command_stream_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotCommandStreamingClient>();
  if (!robot_command_stream_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot command streaming client");
    return false;
  }
  command_stream_service_ = robot_command_stream_resp.response;

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot Command Streaming Client successfully created!");

  // Fill in the parts of the joint streaming command request that are constant.
  auto* joint_cmd = joint_request_.mutable_joint_command();

  joint_cmd->mutable_gains()->mutable_k_q_p()->Clear();
  joint_cmd->mutable_gains()->mutable_k_q_p()->Add(k_q_p_.begin(), k_q_p_.end());
  joint_cmd->mutable_gains()->mutable_k_qd_p()->Clear();
  joint_cmd->mutable_gains()->mutable_k_qd_p()->Add(k_qd_p_.begin(), k_qd_p_.end());

  // Let it extrapolate the command a little
  joint_cmd->mutable_extrapolation_duration()->CopyFrom(
      google::protobuf::util::TimeUtil::NanosecondsToDuration(5 * 1e6));

  // WITHOUT THIS NO COMMANDS WILL BE ACCEPTED!!!!
  ::bosdyn::client::SetRequestHeader("SpotHardware", &joint_request_);

  command_stream_started_ = true;
  return true;
}

void SpotHardware::stop_command_stream() {
  if (!command_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Command stream already stopped");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Stopping Command Stream");
  command_stream_started_ = false;
}

void SpotHardware::send_command(const JointCommands& joint_commands) {
  const std::vector<float>& position = joint_commands.position;
  const std::vector<float>& velocity = joint_commands.velocity;
  const std::vector<float>& load = joint_commands.load;
  const std::vector<float>& k_q_p = joint_commands.k_q_p;
  const std::vector<float>& k_qd_p = joint_commands.k_qd_p;

  // build protobuf
  auto* joint_cmd = joint_request_.mutable_joint_command();

  joint_cmd->mutable_position()->Clear();
  joint_cmd->mutable_position()->Add(position.begin(), position.end());
  joint_cmd->mutable_velocity()->Clear();
  joint_cmd->mutable_velocity()->Add(velocity.begin(), velocity.end());
  joint_cmd->mutable_load()->Clear();
  joint_cmd->mutable_load()->Add(load.begin(), load.end());
  joint_cmd->mutable_gains()->mutable_k_q_p()->Clear();
  joint_cmd->mutable_gains()->mutable_k_q_p()->Add(k_q_p.begin(), k_q_p.end());
  joint_cmd->mutable_gains()->mutable_k_qd_p()->Clear();
  joint_cmd->mutable_gains()->mutable_k_qd_p()->Add(k_qd_p.begin(), k_qd_p.end());

  if (endpoint_ == nullptr) {
    auto endpoint_result = robot_->StartTimeSyncAndGetEndpoint();
    if (!endpoint_result) {
      RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not get timesync endpoint");
      return;
    }
    endpoint_ = endpoint_result.response;
  }

  auto time_point_local = ::bosdyn::common::TimePoint(std::chrono::system_clock::now() + std::chrono::milliseconds(50));

  ::bosdyn::common::RobotTimeConverter converter = endpoint_->GetRobotTimeConverter();
  joint_cmd->mutable_end_time()->CopyFrom(converter.RobotTimestampFromLocal(time_point_local));

  // Send joint stream command
  auto joint_control_stream = command_stream_service_->JointControlStream(joint_request_);
  if (!joint_control_stream) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to send command: '%s'",
                 joint_control_stream.status.DebugString().c_str());
    return;
  }
}

void SpotHardware::release_lease() {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Releasing Lease");
  bosdyn::api::ReturnLeaseRequest msg;
  auto lease_result = robot_->GetWallet()->GetOwnedLeaseProto("body");
  msg.mutable_lease()->CopyFrom(lease_result.response);
  auto resp = lease_client_->ReturnLease(msg);
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Return lease status: %s", resp.status.DebugString().c_str());
}

}  // namespace spot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_hardware_interface::SpotHardware, hardware_interface::SystemInterface)
