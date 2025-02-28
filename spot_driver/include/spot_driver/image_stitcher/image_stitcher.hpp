// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <optional>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/tf_listener_interface_base.hpp>
#include <string>
#include <utility>
#include <vector>

/**
virtual camera intrinsics
{                   //
 385.,   0., 315.,  // increasing fx stretches left-right, cx moves image left
   0., 385., 844.,  // increasing fy zooms in, cy moves image down
   0.,   0.,   1.};

  ros2 run spot_driver image_stitcher_node --ros-args \
    -p body_frame:=Lionel/body \
    -p virtual_camera_frame:=Lionel/virtual_camera \
    -p virtual_camera_intrinsics:="[385., 0., 315., 0., 385., 844., 0., 0., 1.]" \
    -p virtual_camera_projection_plane:="[-0.15916, 0., 0.987253]" \
    -p virtual_camera_plane_distance:=0.5 \
    -p stitched_image_row_padding:=1182 \
    --remap virtual_camera/image:=/Lionel/camera/frontmiddle_virtual/image \
    --remap left/image:=/Lionel/camera/frontleft/image \
    --remap left/camera_info:=/Lionel/camera/frontleft/camera_info \
    --remap right/image:=/Lionel/camera/frontright/image \
    --remap right/camera_info:=/Lionel/camera/frontright/camera_info
 */
namespace spot_ros2 {
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Transform = geometry_msgs::msg::Transform;
using Time = builtin_interfaces::msg::Time;
using DualImageCallbackFn =
    std::function<void(const std::shared_ptr<const Image>&, const std::shared_ptr<const CameraInfo>&,
                       const std::shared_ptr<const Image>&, const std::shared_ptr<const CameraInfo>&)>;

class CameraSynchronizerBase {
 public:
  virtual ~CameraSynchronizerBase() = default;
  virtual void registerCallback(const DualImageCallbackFn& fn) = 0;
};

class RclcppCameraSynchronizer : public CameraSynchronizerBase {
 public:
  explicit RclcppCameraSynchronizer(const std::shared_ptr<rclcpp::Node>& node);

  void registerCallback(const DualImageCallbackFn& fn) override;

 private:
  using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<ApproximateTimePolicy>;

  std::unique_ptr<Synchronizer> sync_;

  image_transport::SubscriberFilter subscriber_image1_;
  message_filters::Subscriber<CameraInfo> subscriber_info1_;
  image_transport::SubscriberFilter subscriber_image2_;
  message_filters::Subscriber<CameraInfo> subscriber_info2_;
};

/**
 * Handles side effects and parameters for virtual camera
 */
class CameraHandleBase {
 public:
  virtual ~CameraHandleBase() = default;
  virtual void publish(const Image& image, const CameraInfo& info) const = 0;
  virtual void broadcast(const Transform& tf, const Time& stamp) = 0;
  virtual std::string getBodyFrame() const = 0;
  virtual std::string getCameraFrame() const = 0;
  virtual cv::Matx33d getIntrinsics() const = 0;
  virtual cv::Vec3d getPlaneNormal() const = 0;
  virtual double getPlaneDistance() const = 0;
  virtual int getRowPadding() const = 0;
};

class RclcppCameraHandle : public CameraHandleBase {
 public:
  explicit RclcppCameraHandle(const std::shared_ptr<rclcpp::Node>& node, const std::string& frame_prefix);

  void publish(const Image& image, const CameraInfo& info) const override;
  void broadcast(const Transform& tf, const Time& stamp) override;
  std::string getBodyFrame() const override;
  std::string getCameraFrame() const override;
  cv::Matx33d getIntrinsics() const override;
  cv::Vec3d getPlaneNormal() const override;
  double getPlaneDistance() const override;
  int getRowPadding() const override;

 private:
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher camera_publisher_;
  RclcppTfBroadcasterInterface tf_broadcaster_;
  std::string body_frame_;
  std::string camera_frame_;
  cv::Matx33d intrinsics_;
  cv::Vec3d plane_normal_;
  double plane_distance_;
  int row_padding_;
};

struct MiddleCamera {
  MiddleCamera(const cv::Matx33d& virtual_intrinsics, const cv::Vec3d& plane_normal, double plane_distance,
               int row_padding, const Transform& body_tform_left, const Transform& body_tform_right,
               const CameraInfo& info_left, const CameraInfo& info_right);
  Image::SharedPtr stitch(const std::shared_ptr<const Image>& left, const std::shared_ptr<const Image>& right);
  Transform getTransform();

 private:
  /* Transforms used to compute the homographies */
  cv::Matx44d body_tform_left_;
  cv::Matx44d body_tform_right_;
  cv::Matx44d body_tform_virtual_;
  std::vector<cv::Matx33d> homography_;
  // Top left corners of each image
  std::vector<cv::Point> corners_;
  // These are where the warped images/masks go. They are in vector form because later
  // they get passed into functions that need them in vector form.
  std::vector<cv::UMat> warped_images_;
  // Warped images encoded in CV_16S and CV_32F, respectively.
  std::vector<cv::UMat> warped_images_f_;
  std::vector<cv::UMat> warped_images_s_;

  std::vector<cv::UMat> warped_masks_;
  std::vector<std::pair<cv::UMat, uchar>> level_masks_;
  cv::UMat blend_mask_;
  cv::UMat result_;
  cv::Size result_size_;
  /* Parts of the stitching pipeline that make the images look good */
  // Color corrects the images between each other
  cv::detail::BlocksGainCompensator compensator_;
  // In the default opencv stitching pipeline they use GraphCut for seam finding
  // which is slower than Dp. In testing, Dp seemed to work well for this use case.
  cv::detail::DpSeamFinder seamer_;
  // Final blending of the two images along the seam line
  cv::detail::MultiBandBlender blender_;
};

class ImageStitcher {
 public:
  ImageStitcher(std::unique_ptr<CameraSynchronizerBase> synchronizer,
                std::unique_ptr<TfListenerInterfaceBase> tf_listener, std::unique_ptr<CameraHandleBase> camera_handle,
                std::unique_ptr<LoggerInterfaceBase> logger,
                std::unique_ptr<ParameterInterfaceBase> parameter_interface);

 private:
  void callback(const std::shared_ptr<const Image>&, const std::shared_ptr<const CameraInfo>&,
                const std::shared_ptr<const Image>&, const std::shared_ptr<const CameraInfo>&);

  std::unique_ptr<CameraSynchronizerBase> synchronizer_;
  std::unique_ptr<TfListenerInterfaceBase> tf_listener_;
  std::unique_ptr<CameraHandleBase> camera_handle_;
  std::unique_ptr<LoggerInterfaceBase> logger_;
  std::unique_ptr<ParameterInterfaceBase> parameter_interface_;

  std::optional<MiddleCamera> camera_;
};
}  // namespace spot_ros2
