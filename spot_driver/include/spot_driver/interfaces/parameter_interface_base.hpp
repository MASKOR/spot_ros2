// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

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
  virtual std::string getSpotName() const = 0;
  virtual std::set<spot_ros2::SpotCamera> getDefaultCamerasUsed(bool has_arm) const = 0;
  virtual tl::expected<std::set<spot_ros2::SpotCamera>, std::string> getCamerasUsed(bool has_arm) const = 0;

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
  static constexpr auto kDefaultPreferredOdomFrame = "odom";
  static constexpr auto kDefaultCamerasUsedWithoutArm = {"frontleft", "frontright", "left", "right", "back"};
  static constexpr auto kDefaultCamerasUsedWithArm = {"frontleft", "frontright", "left", "right", "back", "hand"};
};
}  // namespace spot_ros2
