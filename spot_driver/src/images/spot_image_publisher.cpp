// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/images/spot_image_publisher.hpp>

#include <rmw/qos_profiles.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver/api/default_image_client.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/images/images_middleware_handle.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/types.hpp>

#include <memory>
#include <optional>

namespace {
constexpr auto kImageCallbackPeriod = std::chrono::duration<double>{1.0 / 15.0};  // 15 Hz
constexpr auto kDefaultDepthImageQuality = 100.0;
}  // namespace

namespace spot_ros2::images {
::bosdyn::api::GetImageRequest createImageRequest(const std::set<ImageSource>& sources, const bool has_rgb_cameras,
                                                  const double rgb_image_quality, const bool get_raw_rgb_images) {
  ::bosdyn::api::GetImageRequest request_message;

  for (const auto& source : sources) {
    const auto source_name = toSpotImageSourceName(source);

    if (source.type == SpotImageType::RGB) {
      bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
      image_request->set_image_source_name(source_name);
      // JPEG images can have a user-configurable image quality setting.
      image_request->set_quality_percent(rgb_image_quality);
      // The hand camera always provides RGB images
      if (source.camera == SpotCamera::HAND || has_rgb_cameras) {
        image_request->set_pixel_format(bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8);
        // RGB images can be either raw or JPEG-compressed.
        image_request->set_image_format(get_raw_rgb_images ? bosdyn::api::Image_Format_FORMAT_RAW
                                                           : bosdyn::api::Image_Format_FORMAT_JPEG);
      } else {
        // Grey images are always JPEG-compressed, so selection of RAW has no effect
        image_request->set_pixel_format(bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U8);
        image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_JPEG);
      }
    } else if (source.type == SpotImageType::DEPTH) {
      bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
      image_request->set_image_source_name(source_name);
      image_request->set_quality_percent(kDefaultDepthImageQuality);
      image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
    } else {
      // SpotImageType::DEPTH_REGISTERED
      bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
      image_request->set_image_source_name(source_name);
      image_request->set_quality_percent(kDefaultDepthImageQuality);
      image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
    }
  }

  return request_message;
}

SpotImagePublisher::SpotImagePublisher(const std::shared_ptr<ImageClientInterface>& image_client_interface,
                                       std::unique_ptr<MiddlewareHandle> middleware_handle,
                                       std::unique_ptr<ParameterInterfaceBase> parameters,
                                       std::unique_ptr<LoggerInterfaceBase> logger,
                                       std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                       std::unique_ptr<TimerInterfaceBase> timer, bool has_arm)
    : image_client_interface_{image_client_interface},
      middleware_handle_{std::move(middleware_handle)},
      parameters_{std::move(parameters)},
      logger_{std::move(logger)},
      tf_broadcaster_{std::move(tf_broadcaster)},
      timer_{std::move(timer)},
      has_arm_{has_arm} {}

bool SpotImagePublisher::initialize() {
  // These parameters all fall back to default values if the user did not set them at runtime
  const auto rgb_image_quality = parameters_->getRGBImageQuality();
  const auto publish_rgb_images = parameters_->getPublishRGBImages();
  const auto publish_depth_images = parameters_->getPublishDepthImages();
  const auto publish_depth_registered_images = parameters_->getPublishDepthRegisteredImages();
  const auto has_rgb_cameras = parameters_->getHasRGBCameras();
  // always use compressed transport from SPOT, we decompress it in paralell if desired
  const auto publish_raw_rgb_cameras = false;
  const auto uncompress_images = parameters_->getUncompressImages();
  const auto publish_compressed_images = parameters_->getPublishCompressedImages();

  std::set<spot_ros2::SpotCamera> cameras_used;
  const auto cameras_used_parameter = parameters_->getCamerasUsed(has_arm_);
  if (cameras_used_parameter.has_value()) {
    cameras_used = cameras_used_parameter.value();
  } else {
    logger_->logWarn("Invalid cameras_used parameter! Got error: " + cameras_used_parameter.error() +
                     " Defaulting to publishing from all cameras.");
    cameras_used = parameters_->getDefaultCamerasUsed(has_arm_);
  }

  // Generate the set of image sources based on which cameras the user has requested that we publish
  const auto sources =
      createImageSources(publish_rgb_images, publish_depth_images, publish_depth_registered_images, cameras_used);

  // Generate the image request message to capture the data from the specified image sources
  image_request_message_ = createImageRequest(sources, has_rgb_cameras, rgb_image_quality, publish_raw_rgb_cameras);

  // Create a publisher for each image source
  middleware_handle_->createPublishers(sources, uncompress_images, publish_compressed_images);

  // Create a timer to request and publish images at a fixed rate
  timer_->setTimer(kImageCallbackPeriod, [this, uncompress_images, publish_compressed_images]() {
    timerCallback(uncompress_images, publish_compressed_images);
  });

  return true;
}

void SpotImagePublisher::timerCallback(bool uncompress_images, bool publish_compressed_images) {
  if (!image_request_message_) {
    logger_->logError("No image request message generated. Returning.");
    return;
  }

  const auto image_result =
      image_client_interface_->getImages(*image_request_message_, uncompress_images, publish_compressed_images);
  if (!image_result.has_value()) {
    logger_->logError(std::string{"Failed to get images: "}.append(image_result.error()));
    return;
  }

  middleware_handle_->publishImages(image_result.value().images_, image_result.value().compressed_images_);
  tf_broadcaster_->updateStaticTransforms(image_result.value().transforms_);
}
}  // namespace spot_ros2::images
