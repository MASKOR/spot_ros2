// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <spot_driver/api/spot_image_sources.hpp>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

namespace {
using ::testing::AllOf;
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::IsFalse;
using ::testing::Property;
using ::testing::StrEq;
using ::testing::UnorderedElementsAre;
}  // namespace

namespace spot_ros2::images::test {
TEST(SpotImageSources, toRosTopic) {
  // Check that all combinations of SpotCamera and SpotImageType are converted to the correct topic name

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB}), StrEq("camera/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}), StrEq("camera/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::RGB}), StrEq("camera/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::RGB}), StrEq("camera/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::RGB}), StrEq("camera/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}), StrEq("camera/right"));

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH}), StrEq("depth/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH}), StrEq("depth/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}), StrEq("depth/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}), StrEq("depth/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH}), StrEq("depth/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}), StrEq("depth/right"));

  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/frontleft"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/frontright"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/back"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/hand"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/left"));
  EXPECT_THAT(toRosTopic(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("depth_registered/right"));
}

TEST(SpotImageSources, toSpotImageSourceName) {
  // Check that all combinations of SpotCamera and SpotImageType are converted to the correct Spot API source name

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB}),
              StrEq("frontleft_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}),
              StrEq("frontright_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::RGB}), StrEq("back_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::RGB}), StrEq("left_fisheye_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}), StrEq("right_fisheye_image"));

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH}),
              StrEq("frontleft_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH}),
              StrEq("frontright_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}), StrEq("back_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH}), StrEq("left_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}), StrEq("right_depth"));

  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("frontleft_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("frontright_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED}),
              StrEq("back_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("left_depth_in_visual_frame"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}),
              StrEq("right_depth_in_visual_frame"));

  // Note that the Spot API uses a different naming convention for the hand images than for the body images.
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::RGB}), StrEq("hand_color_image"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}), StrEq("hand_depth"));
  EXPECT_THAT(toSpotImageSourceName(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}),
              StrEq("hand_depth_in_hand_color_frame"));
}

TEST(SpotImageSources, fromSpotImageSourceName) {
  // WHEN we try to convert an invalid source name
  const std::string kInvalidSourceName{"this_is_an_invalid_source_name"};
  // THEN conversion cleanly fails and returns the correct error message.
  EXPECT_THAT(fromSpotImageSourceName(kInvalidSourceName),
              AllOf(Property(&tl::expected<ImageSource, std::string>::has_value, IsFalse()),
                    Property(&tl::expected<ImageSource, std::string>::error,
                             StrEq("Could not convert source name `" + kInvalidSourceName + "` to ImageSource."))));

  // Check that all Spot API source names are converted to the correct pairing of SpotCamera and SpotImageType

  EXPECT_THAT(fromSpotImageSourceName("frontleft_fisheye_image").value(),
              Eq(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB}));
  EXPECT_THAT(fromSpotImageSourceName("frontright_fisheye_image").value(),
              Eq(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}));
  EXPECT_THAT(fromSpotImageSourceName("back_fisheye_image").value(),
              Eq(ImageSource{SpotCamera::BACK, SpotImageType::RGB}));
  EXPECT_THAT(fromSpotImageSourceName("left_fisheye_image").value(),
              Eq(ImageSource{SpotCamera::LEFT, SpotImageType::RGB}));
  EXPECT_THAT(fromSpotImageSourceName("right_fisheye_image").value(),
              Eq(ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}));

  EXPECT_THAT(fromSpotImageSourceName("frontleft_depth").value(),
              Eq(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH}));
  EXPECT_THAT(fromSpotImageSourceName("frontright_depth").value(),
              Eq(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH}));
  EXPECT_THAT(fromSpotImageSourceName("back_depth").value(), Eq(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}));
  EXPECT_THAT(fromSpotImageSourceName("left_depth").value(), Eq(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH}));
  EXPECT_THAT(fromSpotImageSourceName("right_depth").value(), Eq(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}));

  EXPECT_THAT(fromSpotImageSourceName("frontleft_depth_in_visual_frame").value(),
              Eq(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED}));
  EXPECT_THAT(fromSpotImageSourceName("frontright_depth_in_visual_frame").value(),
              Eq(ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}));
  EXPECT_THAT(fromSpotImageSourceName("back_depth_in_visual_frame").value(),
              Eq(ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED}));
  EXPECT_THAT(fromSpotImageSourceName("left_depth_in_visual_frame").value(),
              Eq(ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED}));
  EXPECT_THAT(fromSpotImageSourceName("right_depth_in_visual_frame").value(),
              Eq(ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}));

  // Note that the Spot API uses a different naming convention for the hand images than for the body images.
  EXPECT_THAT(fromSpotImageSourceName("hand_color_image").value(),
              Eq(ImageSource{SpotCamera::HAND, SpotImageType::RGB}));
  EXPECT_THAT(fromSpotImageSourceName("hand_depth").value(), Eq(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}));
  EXPECT_THAT(fromSpotImageSourceName("hand_depth_in_hand_color_frame").value(),
              Eq(ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}));
}

TEST(SpotImageSources, createImageSources) {
  std::set<spot_ros2::SpotCamera> default_cameras_with_arm = {
      spot_ros2::SpotCamera::FRONTLEFT, spot_ros2::SpotCamera::FRONTRIGHT, spot_ros2::SpotCamera::LEFT,
      spot_ros2::SpotCamera::RIGHT,     spot_ros2::SpotCamera::BACK,       spot_ros2::SpotCamera::HAND};
  std::set<spot_ros2::SpotCamera> default_cameras_without_arm = {
      spot_ros2::SpotCamera::FRONTLEFT, spot_ros2::SpotCamera::FRONTRIGHT, spot_ros2::SpotCamera::LEFT,
      spot_ros2::SpotCamera::RIGHT, spot_ros2::SpotCamera::BACK};
  std::set<spot_ros2::SpotCamera> only_front_cameras = {spot_ros2::SpotCamera::FRONTLEFT,
                                                        spot_ros2::SpotCamera::FRONTRIGHT};
  // WHEN no image types are requested, regardless of whether or not the hand camera is requested
  // THEN no ImageSources are returned
  EXPECT_THAT(createImageSources(false, false, false, default_cameras_without_arm), IsEmpty());
  EXPECT_THAT(createImageSources(false, false, false, default_cameras_with_arm), IsEmpty());

  // WHEN RGB images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all RGB body cameras are returned
  EXPECT_THAT(createImageSources(true, false, false, default_cameras_without_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::BACK, SpotImageType::RGB},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::RGB}));

  // WHEN RGB images are requested from the body cameras and the hand camera
  // THEN image sources for all RGB body cameras and the hand camera are returned
  EXPECT_THAT(createImageSources(true, false, false, default_cameras_with_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::BACK, SpotImageType::RGB},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::HAND, SpotImageType::RGB}));

  // WHEN depth images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all depth body cameras are returned
  EXPECT_THAT(createImageSources(false, true, false, default_cameras_without_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}));

  // WHEN depth images are requested from the body cameras and the hand camera
  // THEN image sources for all depth body cameras and the hand camera are returned
  EXPECT_THAT(createImageSources(false, true, false, default_cameras_with_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::BACK, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::HAND, SpotImageType::DEPTH}));

  // WHEN registered depth images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all registered depth body cameras are returned
  EXPECT_THAT(createImageSources(false, false, true, default_cameras_without_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}));

  // WHEN registered depth images are requested from the body cameras and the hand camera
  // THEN image sources for all registered depth body cameras and the hand camera are returned
  EXPECT_THAT(createImageSources(false, false, true, default_cameras_with_arm),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}));

  // WHEN RGB, depth, and registered images are requested from the body cameras, excluding the hand camera
  // THEN image sources for all camera types and all body cameras are returned
  EXPECT_THAT(
      createImageSources(true, true, true, default_cameras_without_arm),
      UnorderedElementsAre(
          ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}, ImageSource{SpotCamera::BACK, SpotImageType::RGB},
          ImageSource{SpotCamera::LEFT, SpotImageType::RGB}, ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
          ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}, ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED}));

  // WHEN RGB, depth, and registered images are requested from the body cameras and the hand camera
  // THEN image sources for all camera types and all body cameras (plus the hand camera) are returned
  EXPECT_THAT(
      createImageSources(true, true, true, default_cameras_with_arm),
      UnorderedElementsAre(
          ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB}, ImageSource{SpotCamera::BACK, SpotImageType::RGB},
          ImageSource{SpotCamera::LEFT, SpotImageType::RGB}, ImageSource{SpotCamera::RIGHT, SpotImageType::RGB},
          ImageSource{SpotCamera::HAND, SpotImageType::RGB}, ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::BACK, SpotImageType::DEPTH}, ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH},
          ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH}, ImageSource{SpotCamera::HAND, SpotImageType::DEPTH},
          ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::BACK, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::LEFT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::RIGHT, SpotImageType::DEPTH_REGISTERED},
          ImageSource{SpotCamera::HAND, SpotImageType::DEPTH_REGISTERED}));

  // WHEN RGB, depth, and registered images are requested from only the frontleft and frontright cameras
  // THEN image sources for only these camera types are returned
  EXPECT_THAT(createImageSources(true, true, true, only_front_cameras),
              UnorderedElementsAre(ImageSource{SpotCamera::FRONTLEFT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::RGB},
                                   ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH},
                                   ImageSource{SpotCamera::FRONTLEFT, SpotImageType::DEPTH_REGISTERED},
                                   ImageSource{SpotCamera::FRONTRIGHT, SpotImageType::DEPTH_REGISTERED}));
}
}  // namespace spot_ros2::images::test
