// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Based off:
// https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/wiggle_arm_example.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spot_ros2_control/spot_joint_map.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum class WiggleState { WIGGLE_DOWN, WIGGLE_MIDDLE, WIGGLE_UP, RESET };

class WiggleArm : public rclcpp::Node {
 public:
  WiggleArm() : Node("wiggle_arm"), wiggle_state_{WiggleState::WIGGLE_DOWN}, initialized_{false} {
    spot_name = declare_parameter("spot_name", "");
    // Indices of the joints to wiggle. This corresponds to WR0 and F1X
    joints_to_wiggle_ = declare_parameter("joints_to_wiggle", std::vector<int>{16, 18});
    // Offset from starting position of each joints_to_wiggle for the first wiggle (up)
    wiggle_up_offsets_ = declare_parameter("wiggle_up_offsets", std::vector<double>{1.0, -0.5});
    // Offset from starting position of each joints_to_wiggle for the second wiggle (down)
    wiggle_down_offsets_ = declare_parameter("wiggle_down_offsets", std::vector<double>{-1.0, -0.5});
    const auto command_rate = declare_parameter("command_rate", 50.0);  // how frequently to send commands in Hz
    const auto seconds_per_motion =
        declare_parameter("seconds_per_motion", 2.0);  // how many seconds each wiggle should take

    points_per_motion_ = static_cast<int>(command_rate * seconds_per_motion);
    njoints_to_wiggle_ = joints_to_wiggle_.size();

    if ((wiggle_up_offsets_.size() != njoints_to_wiggle_) || (wiggle_down_offsets_.size() != njoints_to_wiggle_)) {
      throw std::logic_error("Wiggle offsets must be the same size as joints_to_wiggle!");
    }

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "low_level/joint_states", 10, std::bind(&WiggleArm::joint_states_callback, this, std::placeholders::_1));
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("forward_position_controller/commands", 10);
    const auto timer_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1. / command_rate));
    timer_ = create_wall_timer(timer_period, std::bind(&WiggleArm::timer_callback, this));
  }

 private:
  std::string spot_name;
  // stores joint angles and desired offsets
  std::vector<double> nominal_joint_angles_;
  std::vector<int64_t> joints_to_wiggle_;
  std::vector<double> wiggle_up_offsets_;
  std::vector<double> wiggle_down_offsets_;
  size_t njoints_to_wiggle_;
  // Command to send to the robot
  std_msgs::msg::Float64MultiArray command_;
  // Parameters
  WiggleState wiggle_state_;
  int points_per_motion_;
  bool initialized_;
  int count_;
  // Timer, publishers, subscribers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

  /// @brief Callback for receiving joint states messages used to store the nominal joint angles of the robot
  /// @param msg ROS message containing joint states
  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    if (!initialized_) {
      RCLCPP_INFO_STREAM(get_logger(), "Received starting joint states");
      sensor_msgs::msg::JointState ordered_joint_states;
      bool successful = spot_ros2_control::order_joint_states(spot_name, msg, ordered_joint_states);
      if (successful) {
        nominal_joint_angles_ = ordered_joint_states.position;
        command_.data = nominal_joint_angles_;
        RCLCPP_INFO_STREAM(get_logger(), "Initialized! Robot will begin to move.");
        initialized_ = true;
      }
    }
  }

  /// @brief Rotates from WIGGLE_DOWN -> WIGGLE_MIDDLE -> WIGGLE_UP -> RESET -> WIGGLE_DOWN -> ...
  /// WIGGLE_MIDDLE and RESET are moving towards the same set of nominal joint angles, just from different directions.
  void state_transition() {
    switch (wiggle_state_) {
      case WiggleState::WIGGLE_DOWN:
        wiggle_state_ = WiggleState::WIGGLE_MIDDLE;
        break;
      case WiggleState::WIGGLE_MIDDLE:
        wiggle_state_ = WiggleState::WIGGLE_UP;
        break;
      case WiggleState::WIGGLE_UP:
        wiggle_state_ = WiggleState::RESET;
        break;
      case WiggleState::RESET:
        wiggle_state_ = WiggleState::WIGGLE_DOWN;
        break;
    }
  }

  /// @brief Fills in the command to send to the robot
  /// @param offsets Vector of offsets to apply to joints_to_wiggle.
  /// @param percentage Percentage of the motion (from 0-1) we are at. 0 corresponds to being at nominal_joint_angles_
  /// and 1 corresponds to being at nominal_joint_angles_+offsets, anything in between is calculated as a linear
  /// interpolation between the two.
  void populate_command(const std::vector<double>& offsets, double percentage) {
    // when percentage is 1, fill command with baseline + offsets. When percentage is 0, fill with baseline.
    for (size_t i = 0; i < njoints_to_wiggle_; ++i) {
      const auto joint = joints_to_wiggle_.at(i);
      command_.data.at(joint) = percentage * offsets.at(i) + nominal_joint_angles_.at(joint);
    }
  }

  /// @brief Given the state, fill the command with the appropriate desired joint angles
  /// @param percentage Percentage through the current state/motion we are currently at, from 0-1.
  void populate_command_from_state(double percentage) {
    switch (wiggle_state_) {
      case WiggleState::WIGGLE_DOWN:
        populate_command(wiggle_down_offsets_, percentage);
        break;
      case WiggleState::WIGGLE_MIDDLE:
        populate_command(wiggle_down_offsets_, 1 - percentage);
        break;
      case WiggleState::WIGGLE_UP:
        populate_command(wiggle_up_offsets_, percentage);
        break;
      case WiggleState::RESET:
        populate_command(wiggle_up_offsets_, 1 - percentage);
        break;
    }
  }

  /// @brief Send commands to the robot depending on the current state to ensure a smooth trajectory
  void timer_callback() {
    // Wait to send commands until we have initialized with the starting joint angles
    if (!initialized_) {
      return;
    }
    // Check if we need to switch state
    if (count_ > points_per_motion_) {
      state_transition();
      count_ = 0;
    }
    // Percentage we are through the desired motion
    const double percentage = static_cast<double>(count_) / points_per_motion_;
    // Fill in the command with the appropriate joint angles given the state
    populate_command_from_state(percentage);
    // Publish the command and increment count
    command_pub_->publish(command_);
    count_++;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WiggleArm>());
  rclcpp::shutdown();
  return 0;
}
