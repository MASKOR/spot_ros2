// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/interfaces/rclcpp_publisher_interface.hpp>
#include <tl_expected/expected.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 1;
constexpr auto kImageTopicSuffix = "image";
}  // namespace

namespace spot_ros2 {

RclcppPublisherInterface::RclcppPublisherInterface(const std::shared_ptr<rclcpp::Node>& node)
    : node_{node}, image_transport_{node_} {}

void RclcppPublisherInterface::createPublishers(const std::set<ImageSource>& image_sources) {
  publishers_.clear();

  for (const auto& image_source : image_sources) {
    // Since these topic names do not have a leading `/` character, they will be published within the namespace of the
    // node, which should match the name of the robot. For example, the topic for the front left RGB camera will
    // ultimately appear as `/MyRobotName/camera/frontleft/image`.
    const auto image_topic_name = toRosTopic(image_source) + "/" + kImageTopicSuffix;
    publishers_.try_emplace(image_topic_name,
                            image_transport_.advertiseCamera(image_topic_name, kPublisherHistoryDepth));
  }
}

tl::expected<void, std::string> RclcppPublisherInterface::publish(
    const std::map<ImageSource, ImageWithCameraInfo>& images) {
  for (const auto& [image_source, image_data] : images) {
    const auto image_topic_name = toRosTopic(image_source) + "/" + kImageTopicSuffix;

    try {
      publishers_.at(image_topic_name).publish(image_data.image, image_data.info);
    } catch (const std::out_of_range& e) {
      return tl::make_unexpected("No publisher exists for image topic `" + image_topic_name + "`.");
    }
  }

  return {};
}
}  // namespace spot_ros2
