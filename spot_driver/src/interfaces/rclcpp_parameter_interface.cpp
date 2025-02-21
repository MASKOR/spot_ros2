// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>

#include <chrono>
#include <cstdlib>
#include <type_traits>
#include <vector>

namespace {
constexpr auto kEnvVarNameHostname = "SPOT_IP";
constexpr auto kEnvVarNamePort = "SPOT_PORT";
constexpr auto kEnvVarNameCertificate = "SPOT_CERTIFICATE";
constexpr auto kEnvVarNameUsername = "BOSDYN_CLIENT_USERNAME";
constexpr auto kEnvVarNamePassword = "BOSDYN_CLIENT_PASSWORD";

constexpr auto kParameterNameHostname = "hostname";
constexpr auto kParameterNamePort = "port";
constexpr auto kParameterNameCamerasUsed = "cameras_used";
constexpr auto kParameterNameCertificate = "certificate";
constexpr auto kParameterNameUsername = "username";
constexpr auto kParameterNamePassword = "password";
constexpr auto kParameterNameRGBImageQuality = "image_quality";
constexpr auto kParameterNameHasRGBCameras = "rgb_cameras";
constexpr auto kParameterNamePublishRGBImages = "publish_rgb";
constexpr auto kParameterNameUncompressImages = "uncompress_images";
constexpr auto kParameterNamePublishCompressedImages = "publish_compressed_images";
constexpr auto kParameterNamePublishDepthImages = "publish_depth";
constexpr auto kParameterNamePublishDepthRegisteredImages = "publish_depth_registered";
constexpr auto kParameterPreferredOdomFrame = "preferred_odom_frame";
constexpr auto kParameterTFRoot = "tf_root";
constexpr auto kParameterSpotName = "spot_name";
constexpr auto kParameterFramePrefix = "frame_prefix";
constexpr auto kParameterNameGripperless = "gripperless";
constexpr auto kParameterTimeSyncTimeout = "timesync_timeout";

namespace type_traits {
template <typename, typename = void>
static constexpr bool is_iterable{};
template <class T>
inline static constexpr bool
    is_iterable<T, std::void_t<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end())>> =
        std::is_same_v<decltype(std::declval<T>().begin()), typename T::iterator>&&
            std::is_same_v<decltype(std::declval<T>().end()), typename T::iterator>;
}  // namespace type_traits

/**
 * @brief Get a rclcpp parameter. If the parameter has not been declared, declare it with the provided default value and
 * then return the value.
 *
 * @tparam ParameterT Parameter type.
 * @param node Parameter will be declared and retrieved from this node.
 * @param name Name of the parameter
 * @param default_value Default value of the parameter to use if it was not already declared.
 * @return If the value for the parameter was already set, return the value. Otherwise, return the default value.
 */
template <typename ParameterT>
ParameterT declareAndGetParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name,
                                  const ParameterT& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<ParameterT>(name, default_value);
  }

  return node->get_parameter_or<ParameterT>(name, default_value);
}

/**
 * @brief Try to get an rclcpp parameter. If the parameter has not been declared, declare it.
 *
 * @tparam ParameterT Parameter type.
 * @param node Parameter will be declared and retrieved from this node.
 * @param name Name of the parameter
 * @return If the value for the parameter was set, return it. Otherwise, return none.
 */
template <typename ParameterT>
std::optional<ParameterT> declareAndGetParameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name) {
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue{ParameterT{}}.get_type());
  }

  rclcpp::Parameter parameter;
  if (!node->get_parameter(name, parameter)) {
    return std::nullopt;
  }
  return std::make_optional(parameter.get_value<ParameterT>());
}

/**
 * @brief Get the value of an environment variable, if it has been set.
 *
 * @param name Name of the environment variable.
 * @return If the environment variable was set, return its value. Otherwise, return nullopt.
 */
std::optional<std::string> getEnvironmentVariable(const std::string& name) {
  const auto value = std::getenv(name.c_str());
  if (!value) {
    return std::nullopt;
  }
  return value;
}

/**
 * @brief Attempt to get a value from an environment variable, and if this fails try to get it from a rclcpp parameter
 * instead.
 *
 * @param node Node to use when retrieving the rclcpp parameter.
 * @param env_var_name Name of the environment variable.
 * @param parameter_name Name of the parameter.
 * @return If the environment variable was set to a valid value, parse it. Otherwise, if the rclcpp parameter was set,
 * return its value. If neither the environment variable nor the rclcpp parameter was set, return none.
 */
template <typename ParameterT>
std::optional<ParameterT> getEnvironmentVariableParameterFallback(const std::shared_ptr<rclcpp::Node>& node,
                                                                  const std::string& env_var_name,
                                                                  const std::string& parameter_name) {
  if (const auto env_var_result = getEnvironmentVariable(env_var_name); env_var_result.has_value()) {
    std::istringstream iss{env_var_result.value()};
    ParameterT value;
    iss >> value;
    if (!iss.fail()) {
      return std::make_optional(value);
    }
  }
  return declareAndGetParameter<ParameterT>(node, parameter_name);
}

/**
 * @brief Attempt to get a value from an environment variable, and if this fails get the value from a rclcpp parameter
 * instead.
 *
 * @param node Node to use when retrieving the rclcpp parameter.
 * @param env_var_name Name of the environment variable.
 * @param parameter_name Name of the parameter.
 * @param default_value Default value of the parameter.
 * @return If the environment variable was set, return its value. Otherwise, if the rclcpp parameter was set, return its
 * value. If neither the environment variable nor the rclcpp parameter was set. return the default value.
 */
std::string getEnvironmentVariableParameterFallback(const std::shared_ptr<rclcpp::Node>& node,
                                                    const std::string& env_var_name, const std::string& parameter_name,
                                                    const std::string& default_value) {
  if (const auto env_var_result = getEnvironmentVariable(env_var_name); env_var_result.has_value()) {
    return env_var_result.value();
  }
  return declareAndGetParameter<std::string>(node, parameter_name, default_value);
}

/**
 * @brief Given an input frame string parameter and a set of valid base names, return a new optional string
 * which is guaranteed to be a valid frame option. If the input parameter is not a valid option, std::nullopt is
 * returned instead.
 * @param frame
 * @param base_names
 * @return A new optional string which is guaranteed to be a valid frame option w.r.t. base_names. If the input
 * parameter is not a valid option, std::nullopt is returned instead.
 */
template <typename OptionsT>
static constexpr std::optional<std::string> validateFrameParameter(const std::string& frame,
                                                                   const OptionsT& base_names) {
  static_assert(type_traits::is_iterable<OptionsT>,
                "Trait bound not satisfied for argument 'base_names', type not iterable.");
  static_assert(std::is_convertible_v<typename OptionsT::value_type, std::string>,
                "Trait bound not satisfied for argument 'base_names', iterator values not convertible to string.");

  // Compare the given frame with all valid options and set false if no match is found.
  const bool is_valid = std::find(base_names.begin(), base_names.end(), frame) != base_names.end();

  return is_valid ? std::make_optional(frame) : std::nullopt;
}

}  // namespace

namespace spot_ros2 {

RclcppParameterInterface::RclcppParameterInterface(const std::shared_ptr<rclcpp::Node>& node) : node_{node} {}

std::string RclcppParameterInterface::getHostname() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNameHostname, kParameterNameHostname, kDefaultHostname);
}

std::optional<int> RclcppParameterInterface::getPort() const {
  return getEnvironmentVariableParameterFallback<int>(node_, kEnvVarNamePort, kParameterNamePort);
}

std::string RclcppParameterInterface::getUsername() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNameUsername, kParameterNameUsername, kDefaultUsername);
}

std::string RclcppParameterInterface::getPassword() const {
  return getEnvironmentVariableParameterFallback(node_, kEnvVarNamePassword, kParameterNamePassword, kDefaultPassword);
}

std::optional<std::string> RclcppParameterInterface::getCertificate() const {
  return getEnvironmentVariableParameterFallback<std::string>(node_, kEnvVarNameCertificate, kParameterNameCertificate);
}

double RclcppParameterInterface::getRGBImageQuality() const {
  return declareAndGetParameter<double>(node_, kParameterNameRGBImageQuality, kDefaultRGBImageQuality);
}

bool RclcppParameterInterface::getHasRGBCameras() const {
  return declareAndGetParameter<bool>(node_, kParameterNameHasRGBCameras, kDefaultHasRGBCameras);
}

bool RclcppParameterInterface::getPublishRGBImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishRGBImages, kDefaultPublishRGBImages);
}

bool RclcppParameterInterface::getUncompressImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNameUncompressImages, kDefaultUncompressImages);
}

bool RclcppParameterInterface::getPublishCompressedImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishCompressedImages, kDefaultPublishCompressedImages);
}

bool RclcppParameterInterface::getPublishDepthImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthImages, kDefaultPublishDepthImages);
}

bool RclcppParameterInterface::getPublishDepthRegisteredImages() const {
  return declareAndGetParameter<bool>(node_, kParameterNamePublishDepthRegisteredImages,
                                      kDefaultPublishDepthRegisteredImages);
}

std::string RclcppParameterInterface::getPreferredOdomFrame() const {
  const std::string preferred_odom_frame =
      declareAndGetParameter<std::string>(node_, kParameterPreferredOdomFrame, kDefaultPreferredOdomFrame);
  const std::optional<std::string> valid_preferred_odom_frame =
      validateFrameParameter(preferred_odom_frame, kValidOdomFrameNames);
  if (!valid_preferred_odom_frame.has_value())
    RCLCPP_WARN(node_->get_logger(), "Given preferred odom frame '%s' is not a valid option, defaulting to '%s'.",
                preferred_odom_frame.c_str(), kDefaultPreferredOdomFrame);
  return valid_preferred_odom_frame.value_or(kDefaultPreferredOdomFrame);
}

std::string RclcppParameterInterface::getTFRoot() const {
  const std::string tf_root = declareAndGetParameter<std::string>(node_, kParameterTFRoot, kDefaultTFRoot);
  const std::optional<std::string> valid_tf_root = validateFrameParameter(tf_root, kValidTFRootFrameNames);
  if (!valid_tf_root.has_value())
    RCLCPP_WARN(node_->get_logger(), "Given TF root frame '%s' is not a valid option, defaulting to '%s'.",
                tf_root.c_str(), kDefaultTFRoot);
  return valid_tf_root.value_or(kDefaultTFRoot);
}

std::optional<std::string> RclcppParameterInterface::getFramePrefix() const {
  return declareAndGetParameter<std::string>(node_, kParameterFramePrefix);
}

bool RclcppParameterInterface::getGripperless() const {
  return declareAndGetParameter<bool>(node_, kParameterNameGripperless, kDefaultGripperless);
}

std::chrono::seconds RclcppParameterInterface::getTimeSyncTimeout() const {
  int timeout_seconds =
      declareAndGetParameter<int>(node_, kParameterTimeSyncTimeout,
                                  std::chrono::duration_cast<std::chrono::seconds>(kDefaultTimeSyncTimeout).count());
  return std::chrono::seconds(timeout_seconds);
}

std::set<spot_ros2::SpotCamera> RclcppParameterInterface::getDefaultCamerasUsed(const bool has_arm,
                                                                                const bool gripperless) const {
  const bool has_hand_camera = has_arm && (!gripperless);
  const auto kDefaultCamerasUsed = (has_hand_camera) ? kCamerasWithHand : kCamerasWithoutHand;
  std::set<spot_ros2::SpotCamera> spot_cameras_used;
  for (const auto& camera : kDefaultCamerasUsed) {
    spot_cameras_used.insert(kRosStringToSpotCamera.at(std::string(camera)));
  }
  return spot_cameras_used;
}

tl::expected<std::set<spot_ros2::SpotCamera>, std::string> RclcppParameterInterface::getCamerasUsed(
    const bool has_arm, const bool gripperless) const {
  const bool has_hand_camera = has_arm && (!gripperless);
  const auto kDefaultCamerasUsed = (has_hand_camera) ? kCamerasWithHand : kCamerasWithoutHand;
  const std::vector<std::string> kDefaultCamerasUsedVector(std::begin(kDefaultCamerasUsed),
                                                           std::end(kDefaultCamerasUsed));
  const auto cameras_used_param =
      declareAndGetParameter<std::vector<std::string>>(node_, kParameterNameCamerasUsed, kDefaultCamerasUsedVector);
  std::set<spot_ros2::SpotCamera> spot_cameras_used;
  for (const auto& camera : cameras_used_param) {
    try {
      const auto spot_camera = kRosStringToSpotCamera.at(camera);
      if ((spot_camera == SpotCamera::HAND) && (!has_arm)) {
        return tl::make_unexpected("Cannot add SpotCamera 'hand', the robot does not have an arm!");
      } else if ((spot_camera == SpotCamera::HAND) && gripperless) {
        return tl::make_unexpected("Cannot add SpotCamera 'hand', the robot is gripperless!");
      }
      spot_cameras_used.insert(spot_camera);
    } catch (const std::out_of_range& e) {
      return tl::make_unexpected("Cannot convert camera '" + camera + "' to a SpotCamera.");
    }
  }
  return spot_cameras_used;
}

std::string RclcppParameterInterface::getSpotNameWithFallbackToNamespace() const {
  // We use the explicit robot_name parameter value if provided.
  // If the spot_name parameter is not found explicitly, then we expect it to always match the namespace of this node,
  // minus the leading `/` character.
  try {
    const std::optional<std::string> spot_name = declareAndGetParameter<std::string>(node_, kParameterSpotName);
    return spot_name.value_or(std::string{node_->get_namespace()}.substr(1));
  } catch (const std::out_of_range& e) {
    // get_namespace() should not return an empty string, but we handle this situation just in case.
    // Note that if no namespace was set when creating the node, get_namespace() will return `/`.
    return "";
  }
}

std::string RclcppParameterInterface::getFramePrefixWithDefaultFallback() const {
  const std::string robot_name = getSpotNameWithFallbackToNamespace();
  const std::optional<std::string> frame_prefix = getFramePrefix();
  return frame_prefix.value_or(!robot_name.empty() ? robot_name + "/" : "");
}
}  // namespace spot_ros2
