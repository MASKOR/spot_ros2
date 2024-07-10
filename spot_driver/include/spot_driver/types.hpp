// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn_api_msgs/msg/manipulator_state.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

namespace spot_ros2 {
/** @brief Represents the six different cameras on Spot. */
enum class SpotCamera {
  BACK,
  FRONTLEFT,
  FRONTRIGHT,
  LEFT,
  RIGHT,
  HAND,
};

/**
 * @brief Map from each ROS camera topic name to SpotCamera value.
 */
static const std::unordered_map<std::string, spot_ros2::SpotCamera> kRosStringToSpotCamera{
    {"back", spot_ros2::SpotCamera::BACK},
    {"frontleft", spot_ros2::SpotCamera::FRONTLEFT},
    {"frontright", spot_ros2::SpotCamera::FRONTRIGHT},
    {"hand", spot_ros2::SpotCamera::HAND},
    {"left", spot_ros2::SpotCamera::LEFT},
    {"right", spot_ros2::SpotCamera::RIGHT},
};

/** @brief Represents the three types of images Spot can capture. */
enum class SpotImageType {
  RGB,
  DEPTH,
  DEPTH_REGISTERED,
};

/** @brief Defines the name and type of an image source. */
struct ImageSource {
  /** @brief Name of the image source. */
  SpotCamera camera;

  /** @brief Type of the image source. */
  SpotImageType type;

  /** @brief Allows comparing one ImageSource instance with another. */
  bool operator==(const ImageSource& e) const { return e.camera == camera && e.type == type; }
};

/** @brief Stores an Image message and a corresponding CameraInfo message together. */
struct ImageWithCameraInfo {
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;

  /** @brief Allows comparing one ImageWithCameraInfo instance with another. */
  bool operator==(const ImageWithCameraInfo& e) const { return e.image == image && e.info == info; }
};

/** @brief Stores an CompressedImage message and a corresponding CameraInfo message together. */
struct CompressedImageWithCameraInfo {
  sensor_msgs::msg::CompressedImage image;
  sensor_msgs::msg::CameraInfo info;

  /** @brief Allows comparing one CompressedImageWithCameraInfo instance with another. */
  bool operator==(const CompressedImageWithCameraInfo& e) const { return e.image == image && e.info == info; }
};

}  // namespace spot_ros2

/**
 * @brief Provide a specialization of std::less for ImageSource to allow using ImageSource as a key in std::map.
 */
template <>
struct std::less<spot_ros2::ImageSource> {
  bool operator()(const spot_ros2::ImageSource& lhs, const spot_ros2::ImageSource& rhs) const {
    // If the camera IDs are different, use them for comparison.
    if (lhs.camera != rhs.camera) {
      return lhs.camera < rhs.camera;
    }

    // Compare the type enum to break the tie if the names are identical.
    return lhs.type < rhs.type;
  }
};

/**
 * @brief A struct of ROS message types that define Spot's Robot State
 */
struct RobotStateMessages {
  spot_msgs::msg::BatteryStateArray battery_states;
  spot_msgs::msg::WiFiState wifi_state;
  spot_msgs::msg::FootStateArray foot_state;
  spot_msgs::msg::EStopStateArray estop_states;

  std::optional<sensor_msgs::msg::JointState> maybe_joint_states;
  std::optional<tf2_msgs::msg::TFMessage> maybe_tf;
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> maybe_odom_twist;
  std::optional<nav_msgs::msg::Odometry> maybe_odom;
  std::optional<spot_msgs::msg::PowerState> maybe_power_state;
  std::optional<spot_msgs::msg::SystemFaultState> maybe_system_fault_state;
  std::optional<bosdyn_api_msgs::msg::ManipulatorState> maybe_manipulator_state;
  std::optional<geometry_msgs::msg::Vector3Stamped> maybe_end_effector_force;
  std::optional<spot_msgs::msg::BehaviorFaultState> maybe_behavior_fault_state;
};
