// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/robot_state.pb.h>
#include <bosdyn_api_msgs/msg/manipulator_state.hpp>
#include <functional>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <spot_msgs/msg/battery_state_array.hpp>
#include <spot_msgs/msg/behavior_fault_state.hpp>
#include <spot_msgs/msg/e_stop_state_array.hpp>
#include <spot_msgs/msg/foot_state_array.hpp>
#include <spot_msgs/msg/power_state.hpp>
#include <spot_msgs/msg/system_fault_state.hpp>
#include <spot_msgs/msg/wi_fi_state.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

namespace spot_ros2 {

/**
 * @brief Mapping from the joint names used within the Spot API to the joint names used in the Spot driver and URDF.
 * @details As an example, "fl.hx" is the name of the joint in the Spot API, and "front_left_hip_x" is the name used for
 * this joint within the URDF.
 */
inline const std::map<const std::string, const std::string> kFriendlyJointNames = {
    {"fl.hx", "front_left_hip_x"},  {"fl.hy", "front_left_hip_y"},  {"fl.kn", "front_left_knee"},
    {"fr.hx", "front_right_hip_x"}, {"fr.hy", "front_right_hip_y"}, {"fr.kn", "front_right_knee"},
    {"hl.hx", "rear_left_hip_x"},   {"hl.hy", "rear_left_hip_y"},   {"hl.kn", "rear_left_knee"},
    {"hr.hx", "rear_right_hip_x"},  {"hr.hy", "rear_right_hip_y"},  {"hr.kn", "rear_right_knee"},
    {"arm0.sh0", "arm_sh0"},        {"arm0.sh1", "arm_sh1"},        {"arm0.hr0", "arm_hr0"},
    {"arm0.el0", "arm_el0"},        {"arm0.el1", "arm_el1"},        {"arm0.wr0", "arm_wr0"},
    {"arm0.wr1", "arm_wr1"},        {"arm0.f1x", "arm_f1x"},
};

/**
 * @brief Given an input string and a prefix string which is a substring starting at the beginning of the input string,
 * return a new string which is the difference between the input string and the prefix string.
 * @param input
 * @param prefix
 * @return A new string which is the difference between the input string and the prefix string.
 */
std::string stripPrefix(const std::string& input, const std::string& prefix);

/**
 * @brief Given an input string and a prefix string, return a new string which is the addition of the prefix string and
 * the input string. If the input string already contains the prefix substring at the beginning, the output will be the
 * same as input.
 * @param input
 * @param prefix
 * @return A new string which is the addition of the prefix string and the input string. If the input string already
 * contains the prefix substring at the beginning, the output will be the same as input.
 */
std::string prependPrefix(const std::string& input, const std::string& prefix);

/**
 * @brief Create a BatteryStateArray ROS message by parsing a RobotState message and applying a clock skew to it.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return A BatteryStateArray containing the battery info from the robot state message.
 */
spot_msgs::msg::BatteryStateArray getBatteryStates(const ::bosdyn::api::RobotState& robot_state,
                                                   const google::protobuf::Duration& clock_skew);

/**
 * @brief Create a WiFiState ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @return A WiFiState containing the wifi connection info from the robot state message.
 */
spot_msgs::msg::WiFiState getWifiState(const ::bosdyn::api::RobotState& robot_state);

/**
 * @brief Create a FootStateArray ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @return A FootStateArray containing the foot state info from the robot state message.
 */
spot_msgs::msg::FootStateArray getFootState(const ::bosdyn::api::RobotState& robot_state);

/**
 * @brief Create an EStopStateArray ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return An EStopStateArray message containing the E-Stop state data from the robot state message.
 */
spot_msgs::msg::EStopStateArray getEstopStates(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew);

/**
 * @brief Create a JointState ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @param prefix The prefix to apply to all robot joint names. This corresponds to the name of the robot. It is expected
 * to terminate with `/`.
 * @return If the robot state message contains joint state data, return a JointState message containing this data.
 * Otherwise, return nullopt.
 */
std::optional<sensor_msgs::msg::JointState> getJointStates(const ::bosdyn::api::RobotState& robot_state,
                                                           const google::protobuf::Duration& clock_skew,
                                                           const std::string& prefix);

/**
 * @brief Create a ROS TFMessage by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @param prefix  The prefix to apply to all robot frame IDs. This corresponds to the name of the robot. It is expected
 * to terminate with `/`.
 * @param preferred_base_frame_id Frame ID to use as the base frame of the TF tree. Must be either "odom" or "vision".
 * @return If the robot state message contains a FrameTreeSnapshot and robot frame data, return a TFMessage containing
 * this data. Otherwise, return nullopt.
 */
std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::RobotState& robot_state,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& preferred_base_frame_id);

/**
 * @brief Create a ROS TFMessage by parsing a FrameTreeSnapshot message.
 *
 * @param frame_tree_snapshot Frame tree snapshot from Spot.
 * @param timestamp_robot The robot-relative timestamp to use when assigning timestamps to the headers of the output
 * tramsform messages.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @param prefix  The prefix to apply to all robot frame IDs. This corresponds to the name of the robot. It is expected
 * to terminate with `/`.
 * @param preferred_base_frame_id Frame ID to use as the base frame of the TF tree. Must be either "odom" or "vision".
 * @param frames_to_ignore Set of frames to not include in the TF tree, defaults to an empty set.
 * @return If the input frame tree snapshot contains a non-zero number of entries, return a TFMessage containing
 * this data. Otherwise, return nullopt.
 */
std::optional<tf2_msgs::msg::TFMessage> getTf(const ::bosdyn::api::FrameTreeSnapshot& frame_tree_snapshot,
                                              const google::protobuf::Timestamp& timestamp_robot,
                                              const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                              const std::string& preferred_base_frame_id,
                                              const std::set<std::string, std::less<>>& frames_to_ignore = {});
/**
 * @brief Create an TwistWithCovarianceStamped ROS message representing Spot's body velocity by parsing a RobotState
 * message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return If the robot state message contains the velocity of the Spot's body relative to the odometry frame in its
 * kinematic state, return a TwistWithCovarianceStamped containing this data. Otherwise, return nullopt.
 */
std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> getOdomTwist(const ::bosdyn::api::RobotState& robot_state,
                                                                           const google::protobuf::Duration& clock_skew,
                                                                           const bool is_using_vision);

/**
 * @brief Create an Odometry ROS message representing Spot's pose and velocity relative to a fixed world frame by
 * parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @param prefix The prefix to apply to all robot frame IDs. This corresponds to the name of the robot. It is expected
 * to terminate with `/`.
 * @param is_using_vision Set to 'true' if Spot's preferred base frame is "vision". Otherwise, the preferred base frame
 * will be "odom".
 * @return If the robot state message's kinematic state contains data about its pose and velocity, return an Odometry
 * message containing this data, Otherwise, return nullopt.
 */
std::optional<nav_msgs::msg::Odometry> getOdom(const ::bosdyn::api::RobotState& robot_state,
                                               const google::protobuf::Duration& clock_skew, const std::string& prefix,
                                               const bool is_using_vision);

/**
 * @brief Create a PowerState ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return If the robot state message contains data about Spot's power state, return a PowerState message containing
 * this data. Otherwise, return nullopt.
 */
std::optional<spot_msgs::msg::PowerState> getPowerState(const ::bosdyn::api::RobotState& robot_state,
                                                        const google::protobuf::Duration& clock_skew);

/**
 * @brief Create a SystemFault ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return If the robot state message contains data about Spot's system fault states, return a SystemFaultState message
 * containing this data. Otherwise, return nullopt.
 */
std::optional<spot_msgs::msg::SystemFaultState> getSystemFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                    const google::protobuf::Duration& clock_skew);

/**
 * @brief Create a ManipulatorState ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @return If the robot state message contains data about Spot's manipulator state, return a ManipulatorState message
 * containing this data. Otherwise, return nullopt.
 */
std::optional<bosdyn_api_msgs::msg::ManipulatorState> getManipulatorState(const ::bosdyn::api::RobotState& robot_state);

/**
 * @brief Create a Vector3Stamped ROS message representing the force exerted on Spot's end effector by parsing a
 * RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @param prefix The prefix to apply to all robot frame IDs. This corresponds to the name of the robot. It is expected
 * to terminate with `/`.
 * @return If the robot state message contains data about the force on Spot's end effector, return a Vector3Stamped
 * message representing this force. Otherwise, return nullopt.
 */
std::optional<geometry_msgs::msg::Vector3Stamped> getEndEffectorForce(const ::bosdyn::api::RobotState& robot_state,
                                                                      const google::protobuf::Duration& clock_skew,
                                                                      const std::string& prefix);

/**
 * @brief Create a BehaviorFaultState ROS message by parsing a RobotState message.
 *
 * @param robot_state Robot state message from Spot.
 * @param clock_skew The clock skew reported by Spot at the timepoint when the robot state was created.
 * @return If the robot state message contains data about Spot's behavior fault states, return a BehaviorFaultState
 * message containing this data. Otherwise, return nullopt.
 */
std::optional<spot_msgs::msg::BehaviorFaultState> getBehaviorFaultState(const ::bosdyn::api::RobotState& robot_state,
                                                                        const google::protobuf::Duration& clock_skew);

}  // namespace spot_ros2
