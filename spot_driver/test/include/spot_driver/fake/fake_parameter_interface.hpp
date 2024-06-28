// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/interfaces/parameter_interface_base.hpp>

#include <optional>
#include <string>
#include <vector>

namespace spot_ros2::test {
class FakeParameterInterface : public ParameterInterfaceBase {
 public:
  std::string getHostname() const override { return kExampleHostname; }

  std::optional<int> getPort() const override { return std::nullopt; }

  std::optional<std::string> getCertificate() const override { return std::nullopt; }

  std::string getUsername() const override { return kExampleUsername; }

  std::string getPassword() const override { return kExamplePassword; }

  double getRGBImageQuality() const override { return rgb_image_quality; }

  bool getHasRGBCameras() const override { return has_rgb_cameras; }

  bool getUncompressImages() const override { return uncompress_images; }

  bool getPublishCompressedImages() const override { return publish_compressed_images; }

  bool getPublishRGBImages() const override { return publish_rgb_images; }

  bool getPublishDepthImages() const override { return publish_depth_images; }

  bool getPublishDepthRegisteredImages() const override { return publish_depth_registered_images; }

  std::string getPreferredOdomFrame() const override { return "odom"; }

  std::string getSpotName() const override { return spot_name; }

  std::vector<std::string> getCamerasUsed(const bool has_arm) const override {
    const auto kDefaultCamerasUsed = has_arm ? kDefaultCamerasUsedWithArm : kDefaultCamerasUsedWithoutArm;
    const std::vector<std::string> kDefaultCamerasUsedVector(std::begin(kDefaultCamerasUsed),
                                                             std::end(kDefaultCamerasUsed));
    return kDefaultCamerasUsedVector;
  }

  static constexpr auto kExampleHostname{"192.168.0.10"};
  static constexpr auto kExampleUsername{"spot_user"};
  static constexpr auto kExamplePassword{"hunter2"};

  double rgb_image_quality = ParameterInterfaceBase::kDefaultRGBImageQuality;
  bool has_rgb_cameras = ParameterInterfaceBase::kDefaultHasRGBCameras;
  bool uncompress_images = ParameterInterfaceBase::kDefaultUncompressImages;
  bool publish_compressed_images = ParameterInterfaceBase::kDefaultPublishCompressedImages;
  bool publish_rgb_images = ParameterInterfaceBase::kDefaultPublishRGBImages;
  bool publish_depth_images = ParameterInterfaceBase::kDefaultPublishDepthImages;
  bool publish_depth_registered_images = ParameterInterfaceBase::kDefaultPublishDepthRegisteredImages;
  std::string spot_name;
};
}  // namespace spot_ros2::test
