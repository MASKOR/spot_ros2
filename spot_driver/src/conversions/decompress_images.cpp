// Copyright (c) 2024 The AI Institute LLC. All rights reserved.

#include <bosdyn/api/directory.pb.h>
#include <bosdyn/api/image.pb.h>
#include <cv_bridge/cv_bridge.h>
#include <google/protobuf/duration.pb.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/conversions/decompress_images.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/types.hpp>
#include <std_msgs/msg/header.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

tl::expected<int, std::string> getCvPixelFormat(const bosdyn::api::Image_PixelFormat& format) {
  switch (format) {
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGB_U8: {
      return CV_8UC3;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_RGBA_U8: {
      return CV_8UC4;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U8: {
      return CV_8UC1;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U16: {
      return CV_16UC1;
    }
    case bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_DEPTH_U16: {
      return CV_16UC1;
    }
    default: {
      return tl::make_unexpected("Unknown pixel format.");
    }
  }
}

std_msgs::msg::Header createImageHeader(const bosdyn::api::ImageCapture& image_capture, const std::string& frame_prefix,
                                        const google::protobuf::Duration& clock_skew) {
  std_msgs::msg::Header header;
  header.frame_id = frame_prefix + image_capture.frame_name_image_sensor();
  header.stamp = spot_ros2::robotTimeToLocalTime(image_capture.acquisition_time(), clock_skew);
  return header;
}

tl::expected<sensor_msgs::msg::Image, std::string> getDecompressImageMsg(const bosdyn::api::ImageCapture& image_capture,
                                                                         const std::string& frame_prefix,
                                                                         const google::protobuf::Duration& clock_skew) {
  const auto& image = image_capture.image();
  auto data = image.data();

  const auto header = createImageHeader(image_capture, frame_prefix, clock_skew);
  const auto pixel_format_cv = getCvPixelFormat(image.pixel_format());
  if (!pixel_format_cv) {
    return tl::make_unexpected("Failed to determine pixel format: " + pixel_format_cv.error());
  }

  if (image.format() == bosdyn::api::Image_Format_FORMAT_JPEG) {
    // When the image is JPEG-compressed, it is represented as a 1 x (width * height) row of bytes.
    // First we create a cv::Mat which contains the compressed image data...
    const cv::Mat img_compressed{1, image.rows() * image.cols(), CV_8UC1, &data.front()};
    // Then we decode it to extract the raw image into a new cv::Mat.
    if (image.pixel_format() == bosdyn::api::Image_PixelFormat_PIXEL_FORMAT_GREYSCALE_U8) {
      const cv::Mat img_grey = cv::imdecode(img_compressed, cv::IMREAD_GRAYSCALE);
      if (!img_grey.data) {
        return tl::make_unexpected("Failed to decode JPEG-compressed image.");
      }
      const auto image = cv_bridge::CvImage{header, "mono8", img_grey}.toImageMsg();
      return *image;
    } else {
      const cv::Mat img_bgr = cv::imdecode(img_compressed, cv::IMREAD_COLOR);
      if (!img_bgr.data) {
        return tl::make_unexpected("Failed to decode JPEG-compressed image.");
      }
      const auto image = cv_bridge::CvImage{header, "bgr8", img_bgr}.toImageMsg();
      return *image;
    }
  } else if (image.format() == bosdyn::api::Image_Format_FORMAT_RAW) {
    const cv::Mat img = cv::Mat(image.rows(), image.cols(), pixel_format_cv.value(), &data.front());
    if (!img.data) {
      return tl::make_unexpected("Failed to decode raw-formatted image.");
    }
    const auto image = cv_bridge::CvImage{header, sensor_msgs::image_encodings::TYPE_16UC1, img}.toImageMsg();
    return *image;
  } else if (image.format() == bosdyn::api::Image_Format_FORMAT_RLE) {
    return tl::make_unexpected("Conversion from FORMAT_RLE is not yet implemented.");
  } else {
    return tl::make_unexpected("Unknown image format.");
  }
}
}  // namespace spot_ros2
