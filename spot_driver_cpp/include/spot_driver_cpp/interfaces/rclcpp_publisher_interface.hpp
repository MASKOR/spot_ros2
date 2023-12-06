// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver_cpp/interfaces/publisher_interface_base.hpp>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Implements PublisherInterfaceBase to use rclcpp Publishers.
 */
class RclcppPublisherInterface : public PublisherInterfaceBase {
 public:
  /**
   * @brief The constructor for RclcppPublisherInterface.
   * @param node A shared_ptr to a rclcpp node. RclcppPublisherInterface shares ownership of the shared_ptr.
   */
  explicit RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node);

  /**
   * @brief Populates the image_publishgers_ and info_publishers_ members with image and camera info publishers.
   * @param image_sources Set of ImageSources. A publisher will be created for each ImageSource.
   */
  void createPublishers(const std::set<ImageSource>& image_sources) override;

  /**
   * @brief Publishes image and camera info messages to ROS 2 topics.
   * @param images Map of image sources to image and camera info data.
   * @return If all images were published successfully, returns void. If there was an error, returns an error message.
   */
  tl::expected<void, std::string> publish(const std::map<ImageSource, ImageWithCameraInfo>& images) override;

 private:
  /** @brief rclcpp node use to create the publishers. */
  std::shared_ptr<rclcpp::Node> node_;

  /** @brief ImageTransport instance used to create CameraPublishers. */
  image_transport::ImageTransport image_transport_;

  /** @brief Map between image topic names and camera publishers. */
  std::unordered_map<std::string, image_transport::CameraPublisher> publishers_;
};
}  // namespace spot_ros2
