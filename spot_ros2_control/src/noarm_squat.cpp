// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Based off: https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/noarm_squat.cpp

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spot_ros2_control/spot_joint_map.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum class SquatState { INITIALIZING, SQUATTING, STANDING };
class NoarmSquat : public rclcpp::Node {
 public:
  NoarmSquat()
      : Node("noarm_squat"), squat_state_{SquatState::INITIALIZING}, initialized_{false}, count_{0}, njoints_{12} {
    // The following joint angles are in the order of FL_hip_x, FL_hip_y, FL_knee, FR..., RL..., RR...
    stand_joint_angles_ = declare_parameter(
        "stand_joint_angles",
        std::vector<double>{0.12, 0.72, -1.45, -0.12, 0.72, -1.45, 0.12, 0.72, -1.45, -0.12, 0.72, -1.45});
    squat_joint_angles_ = declare_parameter(
        "squat_joint_angles",
        std::vector<double>{0.15, 1.30, -2.25, -0.15, 1.30, -2.25, 0.15, 1.30, -2.25, -0.15, 1.30, -2.25});
    const auto command_rate = declare_parameter("command_rate", 50.0);  // how frequently to send commands in Hz
    const auto seconds_per_motion =
        declare_parameter("seconds_per_motion", 2.0);  // how many seconds the squat and stand motions should take

    spot_name = declare_parameter("spot_name", "");

    if (stand_joint_angles_.size() != njoints_) {
      throw std::logic_error("Stand joint angles is the wrong size!");
    }
    if (squat_joint_angles_.size() != njoints_) {
      throw std::logic_error("Squat joint angles is the wrong size!");
    }

    points_per_motion_ = static_cast<int>(command_rate * seconds_per_motion);
    command_.data = std::vector<double>(njoints_, 0.0);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("forward_position_controller/commands", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "low_level/joint_states", 10, std::bind(&NoarmSquat::joint_states_callback, this, std::placeholders::_1));

    const auto timer_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1. / command_rate));
    timer_ = create_wall_timer(timer_period, std::bind(&NoarmSquat::timer_callback, this));
  }

 private:
  std::string spot_name;
  // For storing joint angles
  std::vector<double> stand_joint_angles_;
  std::vector<double> squat_joint_angles_;
  std::vector<double> init_joint_angles_;
  // Command we send to the robot
  std_msgs::msg::Float64MultiArray command_;
  // Parameters and info about state
  SquatState squat_state_;
  int points_per_motion_;
  bool initialized_;
  int count_;
  size_t njoints_;
  // RCLCPP publishers, subscribers, timers
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
        init_joint_angles_ = ordered_joint_states.position;
        RCLCPP_INFO_STREAM(get_logger(), "Initialized! Robot will begin to move.");
        initialized_ = true;
      }
    }
  }

  /// @brief We start in INITIALIZING (going from initial joint angles to squatting joint angles).
  /// Then we just switch indefinitely between SQUATTING and SITTING.
  void state_transition() {
    switch (squat_state_) {
      case SquatState::INITIALIZING:
        squat_state_ = SquatState::SQUATTING;
        break;
      case SquatState::SQUATTING:
        squat_state_ = SquatState::STANDING;
        break;
      case SquatState::STANDING:
        squat_state_ = SquatState::SQUATTING;
        break;
    }
  }

  /// @brief Fills in the command to send to the robot
  /// @param baseline Vector of joint angles to start at
  /// @param goal Vector of joint angles to finish at
  /// @param percentage Percentage of the motion (from 0-1) we are at. 0 corresponds to being at baseline
  /// and 1 corresponds to being at goal, anything in between is calculated as a linear interpolation between
  /// the two.
  void populate_command(const std::vector<double>& baseline, const std::vector<double>& goal, double percentage) {
    for (size_t i = 0; i < njoints_; ++i) {
      command_.data.at(i) = percentage * (goal.at(i) - baseline.at(i)) + baseline.at(i);
    }
  }

  /// @brief Given the state, fill the command with the appropriate desired joint angles to ensure a smooth trajectory.
  /// @param percentage Percentage through the current state/motion we are currently at, from 0-1.
  void populate_command_from_state(double percentage) {
    switch (squat_state_) {
      case SquatState::INITIALIZING:
        populate_command(init_joint_angles_, squat_joint_angles_, percentage);
        break;
      case SquatState::SQUATTING:
        populate_command(squat_joint_angles_, stand_joint_angles_, percentage);
        break;
      case SquatState::STANDING:
        populate_command(stand_joint_angles_, squat_joint_angles_, percentage);
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
  rclcpp::spin(std::make_shared<NoarmSquat>());
  rclcpp::shutdown();
  return 0;
}
