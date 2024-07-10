// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/client/image/image_source_names.h>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/types.hpp>
#include <stdexcept>
#include <tl_expected/expected.hpp>
#include <unordered_map>
#include <vector>

namespace {
using ImageSource = spot_ros2::ImageSource;
using SpotCamera = spot_ros2::SpotCamera;
using SpotImageType = spot_ros2::SpotImageType;

/**
 * @brief Array containing the values of SpotCamera which are associated with the cameras on Spot's body.
 * @details This makes it easier to compose collections of robot cameras, since some robots do not have a hand camera.
 */
static const std::array<SpotCamera, 5> kAllSpotBodyCameras = {
    SpotCamera::BACK, SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT, SpotCamera::LEFT, SpotCamera::RIGHT,
};

/**
 * @brief Map from each SpotCamera value to the corresponding Spot SDK BodyCamera value.
 */
static const std::unordered_map<spot_ros2::SpotCamera, bosdyn::client::BodyCamera> kSpotCameraToSdkBodyCamera{
    {SpotCamera::BACK, bosdyn::client::BodyCamera::BACK},
    {SpotCamera::FRONTLEFT, bosdyn::client::BodyCamera::FRONTLEFT},
    {SpotCamera::FRONTRIGHT, bosdyn::client::BodyCamera::FRONTRIGHT},
    {SpotCamera::LEFT, bosdyn::client::BodyCamera::LEFT},
    {SpotCamera::RIGHT, bosdyn::client::BodyCamera::RIGHT},
};

/**
 * @brief Map from each SpotCamera value to the corresponding string used when composing the ROS camera topic names.
 */
static const std::unordered_map<spot_ros2::SpotCamera, std::string> kSpotCameraToRosString{
    {SpotCamera::BACK, "back"}, {SpotCamera::FRONTLEFT, "frontleft"}, {SpotCamera::FRONTRIGHT, "frontright"},
    {SpotCamera::HAND, "hand"}, {SpotCamera::LEFT, "left"},           {SpotCamera::RIGHT, "right"},
};

/**
 * @brief Map from each ImageSource permutation to the corresponding fully-qualified source name used by the Spot API.
 */
static const std::map<ImageSource, std::string> kImageSourceToAPISourceName = []() {
  std::map<ImageSource, std::string> out;
  for (const auto& entry : kAllSpotBodyCameras) {
    out.try_emplace(ImageSource{entry, SpotImageType::RGB},
                    bosdyn::client::GetFisheyeImageName(kSpotCameraToSdkBodyCamera.at(entry)));
    out.try_emplace(ImageSource{entry, SpotImageType::DEPTH},
                    bosdyn::client::GetDepthName(kSpotCameraToSdkBodyCamera.at(entry)));
    out.try_emplace(ImageSource{entry, SpotImageType::DEPTH_REGISTERED},
                    bosdyn::client::GetDepthInVisualName(kSpotCameraToSdkBodyCamera.at(entry)));
  }
  out.try_emplace(ImageSource{SpotCamera::HAND, SpotImageType::RGB}, bosdyn::client::kHandColorImage);
  out.try_emplace(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}, bosdyn::client::kHandDepth);
  out.try_emplace(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED},
                  bosdyn::client::kHandDepthInHandColorFrame);
  return out;
}();

/**
 * @brief Map from each Spot API source name to the corresponding ImageSource.
 */
static const std::map<std::string, ImageSource> kAPISourceNameToImageSource = []() {
  std::map<std::string, ImageSource> out;
  for (const auto& [key, value] : kImageSourceToAPISourceName) {
    out.try_emplace(value, key);
  }
  return out;
}();
}  // namespace

namespace spot_ros2 {
std::string toRosTopic(const ImageSource& image_source) {
  const auto ros_source_name = kSpotCameraToRosString.at(image_source.camera);

  if (image_source.type == SpotImageType::RGB) {
    return std::string("camera").append("/").append(ros_source_name);
  } else if (image_source.type == SpotImageType::DEPTH) {
    return std::string("depth").append("/").append(ros_source_name);
  } else {
    // SpotImageType::DEPTH_REGISTERED
    return std::string("depth_registered").append("/").append(ros_source_name);
  }
}

std::string toSpotImageSourceName(const ImageSource& image_source) {
  return kImageSourceToAPISourceName.at(image_source);
}

tl::expected<ImageSource, std::string> fromSpotImageSourceName(const std::string& source_name) {
  try {
    return kAPISourceNameToImageSource.at(source_name);
  } catch (const std::out_of_range& e) {
    return tl::make_unexpected("Could not convert source name `" + source_name + "` to ImageSource.");
  }
}

std::set<ImageSource> createImageSources(const bool get_rgb_images, const bool get_depth_images,
                                         const bool get_depth_registered_images,
                                         const std::set<SpotCamera>& spot_cameras_used) {
  std::set<ImageSource> sources;
  if (get_rgb_images) {
    for (const auto& camera : spot_cameras_used) {
      sources.insert(ImageSource{camera, SpotImageType::RGB});
    }
  }
  if (get_depth_images) {
    for (const auto& camera : spot_cameras_used) {
      sources.insert(ImageSource{camera, SpotImageType::DEPTH});
    }
  }
  if (get_depth_registered_images) {
    for (const auto& camera : spot_cameras_used) {
      sources.insert(ImageSource{camera, SpotImageType::DEPTH_REGISTERED});
    }
  }
  return sources;
}
}  // namespace spot_ros2
