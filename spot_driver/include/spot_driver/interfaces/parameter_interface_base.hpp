// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <chrono>
#include <optional>
#include <set>
#include <string>
#include <tl_expected/expected.hpp>

#include <spot_driver/types.hpp>

namespace spot_ros2 {
/**
 * @brief Defines an interface for a class that retrieves the user-configured parameters needed to connect to Spot.
 */
class ParameterInterfaceBase {
 public:
  // ParameterInterfaceBase is move-only
  ParameterInterfaceBase() = default;
  ParameterInterfaceBase(ParameterInterfaceBase&& other) = default;
  ParameterInterfaceBase(const ParameterInterfaceBase&) = delete;
  ParameterInterfaceBase& operator=(ParameterInterfaceBase&& other) = default;
  ParameterInterfaceBase& operator=(const ParameterInterfaceBase&) = delete;

  virtual ~ParameterInterfaceBase() = default;

  // These functions retrieve optional parameters, where a default value can be used if the user does not provide a
  // specific value. If the parameter was set, return the value provided by the user. If the parameter was not set,
  // return the default value.
  virtual std::string getHostname() const = 0;
  virtual std::optional<int> getPort() const = 0;
  virtual std::optional<std::string> getCertificate() const = 0;
  virtual std::string getUsername() const = 0;
  virtual std::string getPassword() const = 0;
  virtual double getRGBImageQuality() const = 0;
  virtual bool getHasRGBCameras() const = 0;
  virtual bool getPublishRGBImages() const = 0;
  virtual bool getUncompressImages() const = 0;
  virtual bool getPublishCompressedImages() const = 0;
  virtual bool getPublishDepthImages() const = 0;
  virtual bool getPublishDepthRegisteredImages() const = 0;
  virtual std::string getPreferredOdomFrame() const = 0;
  virtual std::string getTFRoot() const = 0;
  virtual std::optional<std::string> getFramePrefix() const = 0;
  virtual std::string getSpotNameWithFallbackToNamespace() const = 0;
  virtual std::string getFramePrefixWithDefaultFallback() const = 0;
  virtual bool getGripperless() const = 0;
  virtual std::set<spot_ros2::SpotCamera> getDefaultCamerasUsed(bool has_arm, bool gripperless) const = 0;
  virtual tl::expected<std::set<spot_ros2::SpotCamera>, std::string> getCamerasUsed(bool has_arm,
                                                                                    bool gripperless) const = 0;
  virtual std::chrono::seconds getTimeSyncTimeout() const = 0;

 protected:
  // These are the definitions of the default values for optional parameters.
  static constexpr auto kDefaultHostname = "10.0.0.3";
  static constexpr auto kDefaultUsername = "user";
  static constexpr auto kDefaultPassword = "password";
  static constexpr double kDefaultRGBImageQuality{70.0};
  static constexpr bool kDefaultHasRGBCameras{true};
  static constexpr bool kDefaultPublishRGBImages{true};
  static constexpr bool kDefaultUncompressImages{true};
  static constexpr bool kDefaultPublishCompressedImages{false};
  static constexpr bool kDefaultPublishDepthImages{true};
  static constexpr bool kDefaultPublishDepthRegisteredImages{true};
  static constexpr std::array<const char* const, 2> kValidOdomFrameNames{"odom", "vision"};
  static constexpr std::array<const char* const, 3> kValidTFRootFrameNames{"odom", "vision", "body"};
  static constexpr auto kDefaultPreferredOdomFrame = kValidOdomFrameNames[0];
  static constexpr auto kDefaultTFRoot = kValidTFRootFrameNames[0];
  static constexpr bool kDefaultGripperless{false};
  static constexpr auto kCamerasWithoutHand = {"frontleft", "frontright", "left", "right", "back"};
  static constexpr auto kCamerasWithHand = {"frontleft", "frontright", "left", "right", "back", "hand"};
  static constexpr std::chrono::seconds kDefaultTimeSyncTimeout{5};
};
}  // namespace spot_ros2
