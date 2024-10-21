#include "spot_ros2_control/spot_joint_map.hpp"

namespace spot_ros2_control {

std::unordered_map<std::string, size_t> get_namespaced_joint_map(const std::string& spot_name, bool has_arm) {
  const auto default_map = has_arm ? kJointNameToIndexWithArm : kJointNameToIndexWithoutArm;
  if (spot_name.empty()) {
    return default_map;
  }
  std::unordered_map<std::string, size_t> namespaced_map;
  const std::string joint_prefix = spot_name + "/";
  for (const auto& pair : default_map) {
    namespaced_map[joint_prefix + pair.first] = pair.second;
  }
  return namespaced_map;
}

bool order_joint_states(const std::string& spot_name, const sensor_msgs::msg::JointState& input_joint_states,
                        sensor_msgs::msg::JointState& output_joint_states) {
  const auto njoints = input_joint_states.position.size();
  bool has_arm;
  if (njoints == spot_hardware_interface::kNjointsArm) {
    has_arm = true;
  } else if (njoints == spot_hardware_interface::kNjointsNoArm) {
    has_arm = false;
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotJointMap"), "Invalid number of joints: " << njoints);
    return false;
  }

  output_joint_states.name.resize(njoints);
  output_joint_states.position.resize(njoints);
  output_joint_states.velocity.resize(njoints);
  output_joint_states.effort.resize(njoints);

  const auto joint_map = get_namespaced_joint_map(spot_name, has_arm);

  for (size_t i = 0; i < njoints; ++i) {
    // get the joint name
    const auto& joint_name = input_joint_states.name.at(i);
    try {
      const auto joint_index = joint_map.at(joint_name);
      output_joint_states.name.at(joint_index) = joint_name;
      output_joint_states.position.at(joint_index) = input_joint_states.position.at(i);
      output_joint_states.velocity.at(joint_index) = input_joint_states.velocity.at(i);
      output_joint_states.effort.at(joint_index) = input_joint_states.effort.at(i);
    } catch (const std::out_of_range& e) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("SpotJointMap"), "Invalid joint: " << joint_name);
      return false;
    }
  }
  return true;
}

int get_joint_index(const std::string& joint_str, bool has_arm) {
  // Check if the joint_str has a namespace - if so, remove it
  const size_t namespace_pos = joint_str.find("/");
  const std::string joint_name = (namespace_pos != std::string::npos) ? joint_str.substr(namespace_pos + 1) : joint_str;
  const auto joint_map = has_arm ? kJointNameToIndexWithArm : kJointNameToIndexWithoutArm;
  if (joint_map.find(joint_name) == joint_map.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotJointMap"), "Cannot find joint %s in joint map.", joint_name.c_str());
    return -1;
  }
  return joint_map.at(joint_name);
}

}  // namespace spot_ros2_control
