// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

// Note(llee): I think I would like to move this to `action_stack_spot` because so far it's pretty specific to Spot

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <ranges>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spot_hardware_interface/spot_constants.hpp"
#include "spot_ros2_control/spot_joint_map.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointCommandPassthrough : public rclcpp::Node {
 public:
  JointCommandPassthrough() : Node("joint_passthrough") {
    std::string robot_namespace = declare_parameter("robot_namespace", "Spot");
    robot_namespace = robot_namespace.empty() ? "" : robot_namespace + "/";
    std::string joint_state_topic = robot_namespace + declare_parameter("joint_state_topic", "low_level/joint_states");
    std::string joint_commands_topic = robot_namespace + declare_parameter("joint_commands_topic", "joint_commands");
    std::string controller_commands_topic =
        robot_namespace + declare_parameter("controller_commands_topic", "forward_position_controller/commands");

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic, 10, std::bind(&JointCommandPassthrough::joint_states_callback, this, std::placeholders::_1));

    joint_commands_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_commands_topic, 10,
        std::bind(&JointCommandPassthrough::joint_commands_callback, this, std::placeholders::_1));

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_commands_topic, 10);

    spot_command_.data.reserve(spot_hardware_interface::kNjointsArm);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  std_msgs::msg::Float64MultiArray spot_command_;

  int count_ = 0;

  void joint_states_callback(const sensor_msgs::msg::JointState& msg) {
    // Keep track of the current joint positions
    if (msg.name.size() != static_cast<std::vector<int>::size_type>(spot_hardware_interface::kNjointsArm)) {
      RCLCPP_ERROR(get_logger(), "Expected %i dofs, but got %li", spot_hardware_interface::kNjointsArm,
                   msg.name.size());
      return;
    }

    spot_command_.data.clear();
    spot_command_.data.resize(spot_hardware_interface::kNjointsArm);

    for (int i = 0; i < spot_hardware_interface::kNjointsArm; ++i) {
      int joint_idx = spot_ros2_control::get_joint_index(msg.name[i]);
      if (joint_idx < 0) {
        return;
      }

      spot_command_.data[joint_idx] = msg.position[i];
    }
  }

  void joint_commands_callback(const sensor_msgs::msg::JointState& msg) {
    // Update provided joints with their desired

    int command_dofs = msg.name.size();

    for (int i = 0; i < command_dofs; ++i) {
      int joint_idx = spot_ros2_control::get_joint_index(msg.name[i]);
      if (joint_idx < 0) {
        return;
      }
      spot_command_.data[joint_idx] = msg.position[i];
    }

    ++count_;
    RCLCPP_INFO(get_logger(), "Forwarding command %i", count_);

    command_pub_->publish(spot_command_);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandPassthrough>());
  rclcpp::shutdown();
  return 0;
}
