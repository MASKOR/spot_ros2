// Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/api/default_image_client.hpp>

#include <bosdyn/api/directory.pb.h>
#include <bosdyn/api/image.pb.h>
#include <cv_bridge/cv_bridge.h>
#include <google/protobuf/duration.pb.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/conversions/decompress_images.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/types.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tl_expected/expected.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

static const std::set<std::string> kExcludedStaticTfFrames{
    // We exclude the odometry frames from static transforms since they are not static. We can ignore the body
    // frame because it is a child of odom or vision depending on the preferred_odom_frame, and will be published
    // by the non-static transform publishing that is done by the state callback
    "body",
    "odom",
    "vision",

    // Special case handling for hand camera frames that reference the link "arm0.link_wr1" in their transform
    // snapshots. This name only appears in hand camera transform snapshots and is a known bug in the Spot API.
    // We exclude publishing a static transform from arm0.link_wr1 -> body here because it depends
    // on the arm's position and a static transform would fix it to its initial position.
    "arm0.link_wr1",
};

tl::expected<sensor_msgs::msg::CameraInfo, std::string> toCameraInfoMsg(
    const bosdyn::api::ImageResponse& image_response, const std::string& frame_prefix,
    const google::protobuf::Duration& clock_skew) {
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info_msg.height = image_response.shot().image().rows();
  info_msg.width = image_response.shot().image().cols();
  info_msg.header.frame_id = frame_prefix + image_response.shot().frame_name_image_sensor();
  info_msg.header.stamp = spot_ros2::robotTimeToLocalTime(image_response.shot().acquisition_time(), clock_skew);

  // We assume that the camera images have already been corrected for distortion, so the 5 distortion parameters are all
  // zero.
  info_msg.d = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};

  // Set the rectification matrix to identity, since this is not a stereo pair.
  info_msg.r[0] = 1.0;
  info_msg.r[1] = 0.0;
  info_msg.r[2] = 0.0;
  info_msg.r[3] = 0.0;
  info_msg.r[4] = 1.0;
  info_msg.r[5] = 0.0;
  info_msg.r[6] = 0.0;
  info_msg.r[7] = 0.0;
  info_msg.r[8] = 1.0;

  const auto& intrinsics = image_response.source().pinhole().intrinsics();

  // Create the 3x3 intrinsics matrix.
  info_msg.k[0] = intrinsics.focal_length().x();
  info_msg.k[2] = intrinsics.principal_point().x();
  info_msg.k[4] = intrinsics.focal_length().y();
  info_msg.k[5] = intrinsics.principal_point().y();
  info_msg.k[8] = 1.0;

  // All Spot cameras are functionally monocular, so Tx and Ty are not set here.
  info_msg.p[0] = intrinsics.focal_length().x();
  info_msg.p[2] = intrinsics.principal_point().x();
  info_msg.p[5] = intrinsics.focal_length().y();
  info_msg.p[6] = intrinsics.principal_point().y();
  info_msg.p[10] = 1.0;

  return info_msg;
}

std_msgs::msg::Header createImageHeader(const bosdyn::api::ImageCapture& image_capture, const std::string& frame_prefix,
                                        const google::protobuf::Duration& clock_skew) {
  std_msgs::msg::Header header;
  header.frame_id = frame_prefix + image_capture.frame_name_image_sensor();
  header.stamp = spot_ros2::robotTimeToLocalTime(image_capture.acquisition_time(), clock_skew);
  return header;
}

tl::expected<std::vector<geometry_msgs::msg::TransformStamped>, std::string> getImageTransforms(
    const bosdyn::api::ImageResponse& image_response, const std::string& frame_prefix,
    const google::protobuf::Duration& clock_skew) {
  std::vector<geometry_msgs::msg::TransformStamped> out;
  for (const auto& [child_frame_id, transform] :
       image_response.shot().transforms_snapshot().child_to_parent_edge_map()) {
    // Do not publish static transforms for excluded frames
    if (kExcludedStaticTfFrames.count(child_frame_id) > 0) {
      continue;
    }

    // Rename the parent link "arm0.link_wr1" to "link_wr1" as it appears in robot state
    // which is used for publishing dynamic tfs elsewhere. Without this, the hand camera frame
    // positions would never properly update as no other pipelines reference "arm0.link_wr1".
    const auto parent_frame_id =
        (transform.parent_frame_name() == "arm0.link_wr1") ? "arm_link_wr1" : transform.parent_frame_name();

    const auto tform_msg = spot_ros2::toTransformStamped(
        transform.parent_tform_child(), frame_prefix + parent_frame_id, frame_prefix + child_frame_id,
        spot_ros2::robotTimeToLocalTime(image_response.shot().acquisition_time(), clock_skew));

    out.push_back(tform_msg);
  }
  return out;
}

tl::expected<sensor_msgs::msg::CompressedImage, std::string> toCompressedImageMsg(
    const bosdyn::api::ImageCapture& image_capture, const std::string& frame_prefix,
    const google::protobuf::Duration& clock_skew) {
  const auto& image = image_capture.image();
  if (image.format() != bosdyn::api::Image_Format_FORMAT_JPEG) {
    return tl::make_unexpected("Only JPEG image can be sent as ROS2-compressed image. Format is: " +
                               std::to_string(image.format()));
  }

  auto data = image.data();
  sensor_msgs::msg::CompressedImage compressed_image;
  compressed_image.header = createImageHeader(image_capture, frame_prefix, clock_skew);
  compressed_image.format = "jpeg";
  compressed_image.data.insert(compressed_image.data.begin(), data.begin(), data.end());
  return compressed_image;
}
}  // namespace

namespace spot_ros2 {

DefaultImageClient::DefaultImageClient(::bosdyn::client::ImageClient* image_client,
                                       std::shared_ptr<TimeSyncApi> time_sync_api, const std::string& frame_prefix)
    : image_client_{image_client}, time_sync_api_{time_sync_api}, frame_prefix_{frame_prefix} {}

tl::expected<GetImagesResult, std::string> DefaultImageClient::getImages(::bosdyn::api::GetImageRequest request,
                                                                         bool uncompress_images,
                                                                         bool publish_compressed_images) {
  std::shared_future<::bosdyn::client::GetImageResultType> get_image_result_future =
      image_client_->GetImageAsync(request);

  ::bosdyn::client::GetImageResultType get_image_result = get_image_result_future.get();
  if (!get_image_result.status) {
    return tl::make_unexpected("Failed to get images: " + get_image_result.status.DebugString());
  }

  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    return tl::make_unexpected("Failed to get latest clock skew: " + clock_skew_result.error());
  }

  GetImagesResult out;
  for (const auto& image_response : get_image_result.response.image_responses()) {
    const auto& image = image_response.shot().image();
    auto data = image.data();

    const auto info_msg = toCameraInfoMsg(image_response, frame_prefix_, clock_skew_result.value());
    if (!info_msg) {
      return tl::make_unexpected("Failed to convert SDK image response to ROS CameraInfo message: " + info_msg.error());
    }

    const auto& camera_name = image_response.source().name();
    const auto get_source_name_result = fromSpotImageSourceName(camera_name);
    if (!get_source_name_result.has_value()) {
      return tl::make_unexpected("Failed to convert API image source name to ImageSource: " +
                                 get_source_name_result.error());
    }

    if (image.format() == bosdyn::api::Image_Format_FORMAT_JPEG && publish_compressed_images) {
      const auto compressed_image_msg =
          toCompressedImageMsg(image_response.shot(), frame_prefix_, clock_skew_result.value());
      if (!compressed_image_msg) {
        return tl::make_unexpected("Failed to convert SDK image response to ROS Image message: " +
                                   compressed_image_msg.error());
      }
      out.compressed_images_.try_emplace(get_source_name_result.value(),
                                         CompressedImageWithCameraInfo{compressed_image_msg.value(), info_msg.value()});
    }

    if (image.format() != bosdyn::api::Image_Format_FORMAT_JPEG || uncompress_images) {
      const auto image_msg = getDecompressImageMsg(image_response.shot(), frame_prefix_, clock_skew_result.value());
      if (!image_msg) {
        return tl::make_unexpected("Failed to convert SDK image response to ROS Image message: " + image_msg.error());
      }
      out.images_.try_emplace(get_source_name_result.value(), ImageWithCameraInfo{image_msg.value(), info_msg.value()});
    }

    const auto transforms_result = getImageTransforms(image_response, frame_prefix_, clock_skew_result.value());
    if (transforms_result.has_value()) {
      out.transforms_.insert(out.transforms_.end(), transforms_result.value().begin(), transforms_result.value().end());
    } else {
      return tl::make_unexpected("Failed to get image transforms: " + transforms_result.error());
    }
  }

  return out;
}

}  // namespace spot_ros2
