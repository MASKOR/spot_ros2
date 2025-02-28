// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
#include <cv_bridge/cv_bridge.h>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <opencv2/core/types.hpp>
#include <spot_driver/image_stitcher/image_stitcher.hpp>

#include <message_filters/time_synchronizer.h>
#include <opencv2/core/hal/interface.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <std_msgs/msg/detail/header__builder.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>
namespace {
constexpr auto kHistoryDepth = 10;

cv::Vec3d toCvVec3d(const std::vector<double>& flattened) {
  if (flattened.size() != 3) {
    const auto message =
        std::string("Input vector must have exactly 3 elements. Got ") + std::to_string(flattened.size());
    throw std::domain_error(message);
  }
  cv::Vec3d vec;
  vec(0) = flattened[0];
  vec(1) = flattened[1];
  vec(2) = flattened[2];
  return vec;
}

cv::Matx33d toCvMatx33d(const std::vector<double>& flattened) {
  if (flattened.size() != 9) {
    const auto message =
        std::string("Input vector must have exactly 9 elements. Got ") + std::to_string(flattened.size());
    throw std::domain_error(message);
  }
  cv::Matx33d mat;
  mat(0, 0) = flattened[0];
  mat(0, 1) = flattened[1];
  mat(0, 2) = flattened[2];
  mat(1, 0) = flattened[3];
  mat(1, 1) = flattened[4];
  mat(1, 2) = flattened[5];
  mat(2, 0) = flattened[6];
  mat(2, 1) = flattened[7];
  mat(2, 2) = flattened[8];
  return mat;
}

cv::Matx33d toCvMatx33d(const spot_ros2::CameraInfo& info) {
  return {info.k[0], info.k[1], info.k[2],   // fx,  0, cx
          info.k[3], info.k[4], info.k[5],   //  0, fy, cy
          info.k[6], info.k[7], info.k[8]};  // 0,  0,  1
}

cv::Matx44d toCvMatx44d(const cv::Quatd& q, const cv::Vec3d& t) {
  // Initialize to identity for bottom row of homogeneous transform
  cv::Matx44d transform = cv::Matx44d::eye();
  // Copy in rotation
  auto const r = q.toRotMat3x3();
  transform(0, 0) = r(0, 0);
  transform(0, 1) = r(0, 1);
  transform(0, 2) = r(0, 2);
  transform(1, 0) = r(1, 0);
  transform(1, 1) = r(1, 1);
  transform(1, 2) = r(1, 2);
  transform(2, 0) = r(2, 0);
  transform(2, 1) = r(2, 1);
  transform(2, 2) = r(2, 2);
  // Copy in translation
  transform(0, 3) = t(0);
  transform(1, 3) = t(1);
  transform(2, 3) = t(2);

  return transform;
}

cv::Matx44d toCvMatx44d(const geometry_msgs::msg::Transform& tf) {
  const cv::Quatd q{tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z};
  const cv::Vec3d t{tf.translation.x, tf.translation.y, tf.translation.z};
  return toCvMatx44d(q, t);
}

sensor_msgs::msg::CameraInfo toCameraInfo(const builtin_interfaces::msg::Time& stamp, const std::string& frame,
                                          size_t width, size_t height, const cv::Matx33d& K) {
  sensor_msgs::msg::CameraInfo info;
  // Set header with new frame
  info.header.stamp = stamp;
  info.header.frame_id = frame;
  // Set dimensions
  info.width = width;
  info.height = height;
  // Set intrinsics
  info.k = {                            //
            K(0, 0), K(0, 1), K(0, 2),  //
            K(1, 0), K(1, 1), K(1, 2),  //
            K(2, 0), K(2, 1), K(2, 2)};
  // Set the projection matrix (P)
  // P is a 3x4 matrix, so we extend the intrinsic matrix with zeros
  info.p = {                                 //
            K(0, 0), K(0, 1), K(0, 2), 0.0,  //
            K(1, 0), K(1, 1), K(1, 2), 0.0,  //
            K(2, 0), K(2, 1), K(2, 2), 0.0};
  // Set the distortion coefficients
  info.d = std::vector<double>(5, 0.);
  // Typical distortion model
  info.distortion_model = "plumb_bob";

  return info;
}

void convertToRos(const cv::Matx44d& transform, geometry_msgs::msg::Transform& ros_msg) {
  // Extract rotation matrix
  auto const R = transform.get_minor<3, 3>(0, 0);
  cv::Quatd const q = cv::Quatd::createFromRotMat(R);
  ros_msg.rotation.w = q.w;
  ros_msg.rotation.x = q.x;
  ros_msg.rotation.y = q.y;
  ros_msg.rotation.z = q.z;
  // Extract translation
  ros_msg.translation.x = transform(0, 3);
  ros_msg.translation.y = transform(1, 3);
  ros_msg.translation.z = transform(2, 3);
}

void assignTranslation(const cv::Matx31d& t, cv::Matx44d& T) {
  T(0, 3) = t(0);
  T(1, 3) = t(1);
  T(2, 3) = t(2);
}

void assignRotation(const cv::Matx33d& R, cv::Matx44d& T) {
  T(0, 0) = R(0, 0);
  T(0, 1) = R(0, 1);
  T(0, 2) = R(0, 2);

  T(1, 0) = R(1, 0);
  T(1, 1) = R(1, 1);
  T(1, 2) = R(1, 2);

  T(2, 0) = R(2, 0);
  T(2, 1) = R(2, 1);
  T(2, 2) = R(2, 2);
}

cv::Matx33d between(const cv::Matx33d& R1, const cv::Matx33d& R2) {
  // Get the rotation matrix in between R1 and R2
  // First convert rotation matrices to quaternions
  const cv::Quatd q1 = cv::Quatd::createFromRotMat(R1);
  const cv::Quatd q2 = cv::Quatd::createFromRotMat(R2);
  // Then average the quaternions and convert back to rotation matrix
  return cv::Quatd::slerp(q1, q2, 0.5).toRotMat3x3();
}

cv::Matx44d middle(const cv::Matx44d& T1, const cv::Matx44d& T2) {
  // Take the left and right image frames and determine the transform in the middle
  cv::Matx44d T3 = cv::Matx44d::eye();
  assignRotation(between(T1.get_minor<3, 3>(0, 0), T2.get_minor<3, 3>(0, 0)), T3);
  assignTranslation(0.5 * (T1.get_minor<3, 1>(0, 3) + T2.get_minor<3, 1>(0, 3)), T3);
  return T3;
}

// https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo3
cv::Matx33d computeHomography(const cv::Matx33d& Km, const cv::Matx33d& Kc, const cv::Matx44d& cTm,
                              double plane_distance, const cv::Vec3d& plane_normal) {
  const double inverse_distance = 1. / plane_distance;
  //  Compute the amount to skew the rotation according to the plane definition
  const cv::Matx33d Tt = inverse_distance * cTm.get_minor<3, 1>(0, 3) * plane_normal.t();
  // Compute the geometric homography
  const cv::Matx33d Hg = cTm.get_minor<3, 3>(0, 0) + Tt;
  // Compute the image space homography
  cv::Matx33d H = Km * Hg * Kc.inv();
  // Normalize on the scalar element
  H /= H(2, 2);
  return H;
}
}  // namespace

namespace spot_ros2 {

RclcppCameraSynchronizer::RclcppCameraSynchronizer(const std::shared_ptr<rclcpp::Node>& node) {
  // These topics are remapped onto the actual Spot camera topics in the launch file
  subscriber_image1_.subscribe(node.get(), "left/image", "raw");
  subscriber_info1_.subscribe(node, "left/camera_info");
  subscriber_image2_.subscribe(node.get(), "right/image", "raw");
  subscriber_info2_.subscribe(node, "right/camera_info");

  sync_ = std::make_unique<Synchronizer>(ApproximateTimePolicy(kHistoryDepth), subscriber_image1_, subscriber_info1_,
                                         subscriber_image2_, subscriber_info2_);
}

void RclcppCameraSynchronizer::registerCallback(const DualImageCallbackFn& fn) {
  // This must be a bind instead of a lambda because of how registerCallback is templated within message_filters.
  sync_->registerCallback(
      std::bind(fn, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

RclcppCameraHandle::RclcppCameraHandle(const std::shared_ptr<rclcpp::Node>& node, const std::string& frame_prefix)
    : image_transport_{node},
      camera_publisher_{
          image_transport_.advertiseCamera("virtual_camera/image", 1)},  // Remap to actual topic in launch file
      tf_broadcaster_{node} {
  // Name of the frame to relate the virtual camera with respect to
  const auto body_frame_param = node->declare_parameter("body_frame", "body");
  body_frame_ = frame_prefix + body_frame_param;
  // Name of the virtual camera frame to publish
  const auto camera_frame_param = node->declare_parameter("virtual_camera_frame", "virtual_camera");
  camera_frame_ = frame_prefix + camera_frame_param;
  // Get the virtual camera intrinsics, which could fail if the wrong number of parameters are specified in the yaml
  try {
    intrinsics_ = toCvMatx33d(
        node->declare_parameter("virtual_camera_intrinsics", std::vector<double>{1., 0., 0., 0., 1., 0., 0., 0., 1.}));
  } catch (const std::domain_error& e) {
    RCLCPP_ERROR(node->get_logger(), "Virtual camera intrinsics parameter could not be parsed. Check length. %s",
                 e.what());
  }
  // Definition of the plane to project the images onto
  try {
    plane_normal_ =
        toCvVec3d(node->declare_parameter("virtual_camera_projection_plane", std::vector<double>{0., 0., 1.}));
  } catch (const std::domain_error& e) {
    RCLCPP_ERROR(node->get_logger(), "Virtual camera projection plane parameter could not be parsed. Check length. %s",
                 e.what());
  }
  // Distance from the projected plane to the virtual camera
  plane_distance_ = node->declare_parameter("virtual_camera_plane_distance", 1.);
  // Amount to increase the size of the stitched image rows from the original camera image rows
  row_padding_ = node->declare_parameter("stitched_image_row_padding", 0);
}

void RclcppCameraHandle::publish(const Image& image, const CameraInfo& info) const {
  camera_publisher_.publish(image, info);
}

void RclcppCameraHandle::broadcast(const Transform& tf, const Time& stamp) {
  geometry_msgs::msg::TransformStamped tfstamped;
  tfstamped.transform = tf;
  tfstamped.header.stamp = stamp;
  tfstamped.header.frame_id = body_frame_;
  tfstamped.child_frame_id = camera_frame_;
  tf_broadcaster_.updateStaticTransforms({tfstamped});
}

std::string RclcppCameraHandle::getBodyFrame() const {
  return body_frame_;
}

std::string RclcppCameraHandle::getCameraFrame() const {
  return camera_frame_;
}

cv::Matx33d RclcppCameraHandle::getIntrinsics() const {
  return intrinsics_;
}

cv::Vec3d RclcppCameraHandle::getPlaneNormal() const {
  return plane_normal_;
}

double RclcppCameraHandle::getPlaneDistance() const {
  return plane_distance_;
}

int RclcppCameraHandle::getRowPadding() const {
  return row_padding_;
}

MiddleCamera::MiddleCamera(const cv::Matx33d& virtual_intrinsics, const cv::Vec3d& plane_normal, double plane_distance,
                           int row_padding, const Transform& body_tform_left, const Transform& body_tform_right,
                           const CameraInfo& info_left, const CameraInfo& info_right)
    : body_tform_left_{toCvMatx44d(body_tform_left)},
      body_tform_right_{toCvMatx44d(body_tform_right)},
      body_tform_virtual_{middle(toCvMatx44d(body_tform_left), toCvMatx44d(body_tform_right))},
      homography_(2),
      corners_{cv::Point{0, 0}, cv::Point{0, 0}},
      warped_images_(2),
      warped_images_f_(2),
      warped_images_s_(2),
      warped_masks_(2),
      level_masks_(2),
      result_size_{static_cast<int>(info_left.width), static_cast<int>(info_left.height) + row_padding} {
  /**
   * The math behind these homography computations for the virtual camera can be found here
   * https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo3
   * though the link describes projecting one camera image into the frame of another existing
   * camera image, while here we have to create the transforms for a virtual camera between
   * the left and right images.
   */
  // Note: These computations are very sensitive to significant figures (precision)
  //       Refactoring this code can lead to changes in the value of the double after
  //       the fifth decimal point and should be tested incrementally
  const cv::Matx44d left_tform_virtual = body_tform_left_.inv() * body_tform_virtual_;
  const cv::Matx44d right_tform_virtual = body_tform_right_.inv() * body_tform_virtual_;
  const auto left_intrinsics = toCvMatx33d(info_right);
  const auto right_intrinsics = toCvMatx33d(info_left);
  homography_[0] =
      computeHomography(virtual_intrinsics, left_intrinsics, left_tform_virtual, plane_distance, plane_normal);
  homography_[1] =
      computeHomography(virtual_intrinsics, right_intrinsics, right_tform_virtual, plane_distance, plane_normal);

  // Warp white masks the size of the image using their homographies
  const cv::Size input_size{static_cast<int>(info_left.width), static_cast<int>(info_left.height)};
  for (size_t ndx = 0; ndx < warped_masks_.size(); ++ndx) {
    cv::warpPerspective(cv::UMat{input_size, CV_8U, 255}, warped_masks_[ndx], homography_[ndx], result_size_);
  }
  // Prepare level masks for color compensator
  for (size_t ndx = 0; ndx < warped_masks_.size(); ++ndx) {
    level_masks_[ndx] = std::make_pair(warped_masks_[ndx], 255);
  }
}

Image::SharedPtr MiddleCamera::stitch(const std::shared_ptr<const Image>& left,
                                      const std::shared_ptr<const Image>& right) {
  // Convert the images into a format the can be used by opencv.
  // While the image is coming from the camera on the left of the robot, it sees the right side
  // of the scene and vice versa. This may need to be extracted if this code is to be generalized
  // for something other than the Boston Dynamics Spot Robot, as well as checking the homographies.
  const auto scene_right = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::BGR8);
  const auto scene_left = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::BGR8);

  // Transform the images into the virtual center camera space
  cv::warpPerspective(scene_left->image, warped_images_[0], homography_[0], result_size_);
  cv::warpPerspective(scene_right->image, warped_images_[1], homography_[1], result_size_);

  // Color compensate the images so they blend better
  compensator_.feed(corners_, warped_images_, level_masks_);
  for (size_t ndx = 0; ndx < warped_images_.size(); ndx++) {
    compensator_.apply(ndx, corners_[ndx], warped_images_[ndx], warped_masks_);
  }

  // Create seam masks for the two images to find the best path to blend them
  // Convert images to a different colorspace for seaming
  for (size_t ndx = 0; ndx < warped_images_.size(); ndx++) {
    warped_images_[ndx].convertTo(warped_images_f_[ndx], CV_32F);
  }
  // Find optimal seams to cut at
  seamer_.find(warped_images_f_, corners_, warped_masks_);

  // Blend the images together around the seam
  // Tell the blender to consider the whole warped image for blending
  blender_.prepare(cv::Rect{cv::Point{0, 0}, result_size_});
  // Convert images to a different colorspace for blending
  for (size_t ndx = 0; ndx < warped_images_.size(); ndx++) {
    warped_images_[ndx].convertTo(warped_images_s_[ndx], CV_16S);
  }
  // Feed the warped images and their masks to the blender
  blender_.feed(warped_images_s_[0], warped_masks_[0], cv::Point{0, 0});
  blender_.feed(warped_images_s_[1], warped_masks_[1], cv::Point{0, 0});
  blender_.blend(result_, blend_mask_);

  // Convert the image back to the BGR color space
  result_.convertTo(result_, CV_8U);
  // Return the image in a format that can be published
  return cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", result_.getMat(cv::ACCESS_READ)).toImageMsg();
}

Transform MiddleCamera::getTransform() {
  geometry_msgs::msg::Transform msg;
  convertToRos(body_tform_virtual_, msg);
  return msg;
}

ImageStitcher::ImageStitcher(std::unique_ptr<CameraSynchronizerBase> synchronizer,
                             std::unique_ptr<TfListenerInterfaceBase> tf_listener,
                             std::unique_ptr<CameraHandleBase> camera_handle,
                             std::unique_ptr<LoggerInterfaceBase> logger,
                             std::unique_ptr<ParameterInterfaceBase> parameter_interface)
    : synchronizer_{std::move(synchronizer)},
      tf_listener_{std::move(tf_listener)},
      camera_handle_{std::move(camera_handle)},
      logger_{std::move(logger)},
      parameter_interface_{std::move(parameter_interface)} {
  synchronizer_->registerCallback(
      [this](const std::shared_ptr<const Image>& image_left, const std::shared_ptr<const CameraInfo>& info_left,
             const std::shared_ptr<const Image>& image_right, const std::shared_ptr<const CameraInfo>& info_right) {
        callback(image_left, info_left, image_right, info_right);
      });
}

void ImageStitcher::callback(const std::shared_ptr<const Image>& image_left,
                             const std::shared_ptr<const CameraInfo>& info_left,
                             const std::shared_ptr<const Image>& image_right,
                             const std::shared_ptr<const CameraInfo>& info_right) {
  // The transforms and camera info are assumed to be static, so we only need to lookup these
  // things once, and use them to initialize the stitching camera. It cannot stitch without these
  // parameters so if we can't get them we have to exit the callback.
  if (!camera_.has_value()) {
    const auto body_frame = camera_handle_->getBodyFrame();
    const auto body_tform_left =
        tf_listener_->lookupTransform(info_left->header.frame_id, body_frame, info_left->header.stamp);
    const auto body_tform_right =
        tf_listener_->lookupTransform(info_right->header.frame_id, body_frame, info_right->header.stamp);
    if (!body_tform_left || !body_tform_right) {
      if (!body_tform_left) {
        logger_->logWarn("Valid transform for image frame " + info_left->header.frame_id + " to " + body_frame +
                         " could not be found");
      }
      if (!body_tform_right) {
        logger_->logWarn("Valid transform for image frame " + info_right->header.frame_id + " to " + body_frame +
                         " could not be found");
      }
      return;
    }
    // Build the stitching camera
    camera_ = MiddleCamera{camera_handle_->getIntrinsics(),
                           camera_handle_->getPlaneNormal(),
                           camera_handle_->getPlaneDistance(),
                           camera_handle_->getRowPadding(),
                           body_tform_left->transform,
                           body_tform_right->transform,
                           *info_left,
                           *info_right};
    // Virtual camera transform only has to be broadcast once since it is static wrt the body
    camera_handle_->broadcast(camera_->getTransform(), info_left->header.stamp);
  }
  const auto& current_stamp = info_left->header.stamp;
  const auto& camera_frame = camera_handle_->getCameraFrame();
  // The rest of the time we should just be stitching and publishing
  const auto image_stitched = camera_->stitch(image_left, image_right);
  image_stitched->header.stamp = current_stamp;
  image_stitched->header.frame_id = camera_frame;
  // The only reason we have to remake this every time is to update the time stamp
  const auto info_stitched = toCameraInfo(current_stamp, camera_frame, image_stitched->width, image_stitched->height,
                                          camera_handle_->getIntrinsics());
  camera_handle_->publish(*image_stitched, info_stitched);
}

}  // namespace spot_ros2
