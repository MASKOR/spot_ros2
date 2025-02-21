// Copyright (c) 2024 The AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/directory.pb.h>
#include <bosdyn/api/image.pb.h>
#include <google/protobuf/duration.pb.h>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver/api/spot_image_sources.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2 {

tl::expected<int, std::string> getCvPixelFormat(const bosdyn::api::Image_PixelFormat& format);
std_msgs::msg::Header createImageHeader(const bosdyn::api::ImageCapture& image_capture, const std::string& frame_prefix,
                                        const google::protobuf::Duration& clock_skew);
tl::expected<sensor_msgs::msg::Image, std::string> getDecompressImageMsg(const bosdyn::api::ImageCapture& image_capture,
                                                                         const std::string& frame_prefix,
                                                                         const google::protobuf::Duration& clock_skew);

}  // namespace spot_ros2
