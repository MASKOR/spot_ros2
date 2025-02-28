// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <gmock/gmock.h>

#include <cstdlib>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/rclcpp_test.hpp>

#include <chrono>
#include <memory>

namespace {
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::IsFalse;
using ::testing::IsTrue;
using ::testing::Optional;
using ::testing::StrEq;
using ::testing::UnorderedElementsAre;

constexpr auto kNodeName = "my_node_name";
constexpr auto kNamespace = "my_namespace";

static constexpr auto kSpotName = "my_spot_name";
static constexpr auto kFramePrefix = "some_random_prefix_/_";
static constexpr auto kFramePrefixSeparator = "/";

constexpr auto kEnvVarNameHostname = "SPOT_IP";
constexpr auto kEnvVarNamePort = "SPOT_PORT";
constexpr auto kEnvVarNameCertificate = "SPOT_CERTIFICATE";
constexpr auto kEnvVarNameUsername = "BOSDYN_CLIENT_USERNAME";
constexpr auto kEnvVarNamePassword = "BOSDYN_CLIENT_PASSWORD";
}  // namespace

namespace spot_ros2::test {
class RclcppParameterInterfaceTest : public RclcppTest {};

/**
 * @brief Test fixture that caches any previously-set environment variables which would be overwritten by the test case
 * and restores them after the test is finished.
 * @details This is to prevent interfering with the user's environment if they are running the Spot driver nodes in the
 * same environment as these tests.
 */
class RclcppParameterInterfaceEnvVarTest : public RclcppParameterInterfaceTest {
 public:
  void SetUp() override {
    RclcppTest::SetUp();

    // Get current values of these environment variables.
    const auto hostname = std::getenv(kEnvVarNameHostname);
    const auto port = std::getenv(kEnvVarNamePort);
    const auto certificate = std::getenv(kEnvVarNameCertificate);
    const auto username = std::getenv(kEnvVarNameUsername);
    const auto password = std::getenv(kEnvVarNamePassword);

    // If any are already set, cache them in private members.
    if (hostname) {
      spot_hostname_env_var_cached_ = hostname;
    }
    if (port) {
      spot_port_env_var_cached_ = port;
    }
    if (certificate) {
      spot_certificate_env_var_cached_ = certificate;
    }
    if (username) {
      spot_username_env_var_cached_ = username;
    }
    if (password) {
      spot_password_env_var_cached_ = password;
    }

    // Unset the values of the environment variables to create a clean environment for the test cases.
    unsetenv(kEnvVarNameHostname);
    unsetenv(kEnvVarNamePort);
    unsetenv(kEnvVarNameCertificate);
    unsetenv(kEnvVarNameUsername);
    unsetenv(kEnvVarNamePassword);

    // All test cases that use this fixture also use a node, so create a default node here too.
    node_ = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  }

  void TearDown() override {
    RclcppTest::TearDown();

    // Restore any cached environment variables.
    if (spot_hostname_env_var_cached_) {
      // Calling setenv with the last parameter == 1 replaces any existing value of the environment variable with the
      // new value.
      setenv(kEnvVarNameHostname, spot_hostname_env_var_cached_->c_str(), 1);
    }
    if (spot_port_env_var_cached_) {
      setenv(kEnvVarNamePort, spot_port_env_var_cached_->c_str(), 1);
    }
    if (spot_certificate_env_var_cached_) {
      setenv(kEnvVarNameCertificate, spot_certificate_env_var_cached_->c_str(), 1);
    }
    if (spot_username_env_var_cached_) {
      setenv(kEnvVarNameUsername, spot_username_env_var_cached_->c_str(), 1);
    }
    if (spot_password_env_var_cached_) {
      setenv(kEnvVarNamePassword, spot_password_env_var_cached_->c_str(), 1);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;

 private:
  std::optional<std::string> spot_hostname_env_var_cached_;
  std::optional<std::string> spot_port_env_var_cached_;
  std::optional<std::string> spot_certificate_env_var_cached_;
  std::optional<std::string> spot_username_env_var_cached_;
  std::optional<std::string> spot_password_env_var_cached_;
};

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithNamespace) {
  // GIVEN we create a rclcpp node with a namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotNameWithFallbackToNamespace
  // THEN the parameter interface returns the namespace of the node
  EXPECT_THAT(parameter_interface.getSpotNameWithFallbackToNamespace(), StrEq(kNamespace));
}

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithEmptyNamespace) {
  // GIVEN we create a rclcpp node with an empty namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName, "");
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotNameWithFallbackToNamespace
  // THEN the parameter interface returns an empty string
  EXPECT_THAT(parameter_interface.getSpotNameWithFallbackToNamespace(), IsEmpty());
}

TEST_F(RclcppParameterInterfaceTest, GetSpotNameWithDefaultNamespace) {
  // GIVEN we create a rclcpp node and do not set a specific namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we call getSpotNameWithFallbackToNamespace
  // THEN the parameter interface returns an empty string
  EXPECT_THAT(parameter_interface.getSpotNameWithFallbackToNamespace(), IsEmpty());
}

TEST_F(RclcppParameterInterfaceTest, GetSpotNameFromExplicitParameter) {
  // GIVEN we create rclcpp nodes with and without a specific namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  const auto namespaced_node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we set the spot name config parameter to an explicit value
  node->declare_parameter("spot_name", kSpotName);
  namespaced_node->declare_parameter("spot_name", kSpotName);

  // GIVEN we create a RclcppParameterInterface using these nodes
  RclcppParameterInterface parameter_interface_a{node};
  RclcppParameterInterface parameter_interface_b{namespaced_node};

  // WHEN we call getSpotNameWithFallbackToNamespace
  // THEN the parameter interface returns the explicit spot name in both cases
  EXPECT_THAT(parameter_interface_a.getSpotNameWithFallbackToNamespace(), StrEq(kSpotName));
  EXPECT_THAT(parameter_interface_b.getSpotNameWithFallbackToNamespace(), StrEq(kSpotName));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigFromEnvVars) {
  // GIVEN we declare environment variables for Spot's hostname, username, and password
  constexpr auto hostname_env_var = "10.0.20.5";
  setenv(kEnvVarNameHostname, hostname_env_var, 1);
  constexpr auto port_env_var = "12345";
  setenv(kEnvVarNamePort, port_env_var, 1);
  constexpr auto certificate_env_var = "some/certificate.crt";
  setenv(kEnvVarNameCertificate, certificate_env_var, 1);
  constexpr auto username_env_var = "some_username";
  setenv(kEnvVarNameUsername, username_env_var, 1);
  constexpr auto password_env_var = "very_secure_password";
  setenv(kEnvVarNamePassword, password_env_var, 1);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call getHostname(), getPort(), getCertificate(), getUsername(), and getPassword()
  // THEN the returned values match the values we set on the environment variables
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_env_var));
  EXPECT_THAT(parameter_interface.getPort(), Optional(std::stoi(port_env_var)));
  EXPECT_THAT(parameter_interface.getCertificate(), Optional(certificate_env_var));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_env_var));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_env_var));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigFromParameters) {
  // GIVEN we set all Spot config parameters to values which are different than the default values
  constexpr auto hostname_parameter = "192.168.100.10";
  node_->declare_parameter("hostname", hostname_parameter);
  constexpr auto port_parameter = 12345;
  node_->declare_parameter("port", port_parameter);
  constexpr auto certificate_parameter = "some/certificate.crt";
  node_->declare_parameter("certificate", certificate_parameter);
  constexpr auto username_parameter = "a_different_username";
  node_->declare_parameter("username", username_parameter);
  constexpr auto password_parameter = "other_password";
  node_->declare_parameter("password", password_parameter);
  constexpr auto rgb_image_quality_parameter = 42.0;
  node_->declare_parameter("image_quality", rgb_image_quality_parameter);
  constexpr auto has_rgb_cameras_parameter = false;
  node_->declare_parameter("rgb_cameras", has_rgb_cameras_parameter);
  constexpr auto uncompress_images = false;
  node_->declare_parameter("uncompress_images", uncompress_images);
  constexpr auto publish_compressed_images = true;
  node_->declare_parameter("publish_compressed_images", publish_compressed_images);
  constexpr auto publish_rgb_images_parameter = false;
  node_->declare_parameter("publish_rgb", publish_rgb_images_parameter);
  constexpr auto publish_depth_images_parameter = false;
  node_->declare_parameter("publish_depth", publish_depth_images_parameter);
  constexpr auto publish_depth_registered_images_parameter = false;
  node_->declare_parameter("publish_depth_registered", publish_depth_registered_images_parameter);
  constexpr auto tf_root_parameter = "body";
  node_->declare_parameter("tf_root", tf_root_parameter);
  constexpr auto preferred_odom_frame_parameter = "vision";
  node_->declare_parameter("preferred_odom_frame", preferred_odom_frame_parameter);
  node_->declare_parameter("frame_prefix", kFramePrefix);
  constexpr auto timesync_timeout_parameter = 42;
  node_->declare_parameter("timesync_timeout", timesync_timeout_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN the returned values all match the values we used when declaring the parameters
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_parameter));
  EXPECT_THAT(parameter_interface.getPort(), Optional(port_parameter));
  EXPECT_THAT(parameter_interface.getCertificate(), Optional(certificate_parameter));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_parameter));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_parameter));
  EXPECT_THAT(parameter_interface.getRGBImageQuality(), Eq(rgb_image_quality_parameter));
  EXPECT_THAT(parameter_interface.getHasRGBCameras(), Eq(has_rgb_cameras_parameter));
  EXPECT_THAT(parameter_interface.getUncompressImages(), Eq(uncompress_images));
  EXPECT_THAT(parameter_interface.getPublishCompressedImages(), Eq(publish_compressed_images));
  EXPECT_THAT(parameter_interface.getPublishRGBImages(), Eq(publish_rgb_images_parameter));
  EXPECT_THAT(parameter_interface.getPublishDepthImages(), Eq(publish_depth_images_parameter));
  EXPECT_THAT(parameter_interface.getPublishDepthRegisteredImages(), Eq(publish_depth_registered_images_parameter));
  EXPECT_THAT(parameter_interface.getTFRoot(), Eq(tf_root_parameter));
  EXPECT_THAT(parameter_interface.getPreferredOdomFrame(), StrEq(preferred_odom_frame_parameter));
  EXPECT_THAT(parameter_interface.getFramePrefix(), Optional(kFramePrefix));
  EXPECT_THAT(parameter_interface.getTimeSyncTimeout(), Eq(std::chrono::seconds(timesync_timeout_parameter)));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSpotConfigEnvVarsOverruleParameters) {
  // GIVEN we declare a environment variables for Spot's hostname, username, and password
  constexpr auto hostname_env_var = "10.0.20.5";
  setenv(kEnvVarNameHostname, hostname_env_var, 1);
  constexpr auto port_env_var = "12345";
  setenv(kEnvVarNamePort, port_env_var, 1);
  constexpr auto certificate_env_var = "some/certificate.crt";
  setenv(kEnvVarNameCertificate, certificate_env_var, 1);
  constexpr auto username_env_var = "some_username";
  setenv(kEnvVarNameUsername, username_env_var, 1);
  constexpr auto password_env_var = "very_secure_password";
  setenv(kEnvVarNamePassword, password_env_var, 1);

  // GIVEN we set parameters with different values for Spot's hostname, username, and password
  constexpr auto hostname_parameter = "192.168.100.10";
  node_->declare_parameter("hostname", hostname_parameter);
  constexpr auto port_parameter = 12345;
  node_->declare_parameter("port", port_parameter);
  constexpr auto certificate_parameter = "some/certificate.crt";
  node_->declare_parameter("certificate", certificate_parameter);
  constexpr auto username_parameter = "a_different_username";
  node_->declare_parameter("username", username_parameter);
  constexpr auto password_parameter = "other_password";
  node_->declare_parameter("password", password_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we call getHostname(), getUsername(), and getPassword()
  // THEN the returned values match the values we used when declaring the environment variables, since we expect that
  // the environment variables take precedence over the parameters.
  EXPECT_THAT(parameter_interface.getHostname(), StrEq(hostname_env_var));
  EXPECT_THAT(parameter_interface.getPort(), Optional(std::stoi(port_env_var)));
  EXPECT_THAT(parameter_interface.getCertificate(), Optional(certificate_env_var));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq(username_env_var));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq(password_env_var));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetConfigDefaults) {
  // GIVEN we do not set any environment variables or parameters
  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // WHEN we get the values from the parameter interface
  // THEN the returned values match the expected default values
  EXPECT_THAT(parameter_interface.getHostname(), StrEq("10.0.0.3"));
  EXPECT_THAT(parameter_interface.getPort(), Eq(std::nullopt));
  EXPECT_THAT(parameter_interface.getCertificate(), Eq(std::nullopt));
  EXPECT_THAT(parameter_interface.getUsername(), StrEq("user"));
  EXPECT_THAT(parameter_interface.getPassword(), StrEq("password"));
  EXPECT_THAT(parameter_interface.getRGBImageQuality(), Eq(70.0));
  EXPECT_THAT(parameter_interface.getHasRGBCameras(), IsTrue());
  EXPECT_THAT(parameter_interface.getUncompressImages(), true);
  EXPECT_THAT(parameter_interface.getPublishCompressedImages(), false);
  EXPECT_THAT(parameter_interface.getPublishRGBImages(), IsTrue());
  EXPECT_THAT(parameter_interface.getPublishDepthImages(), IsTrue());
  EXPECT_THAT(parameter_interface.getPublishDepthRegisteredImages(), IsTrue());
  EXPECT_THAT(parameter_interface.getTFRoot(), StrEq("odom"));
  EXPECT_THAT(parameter_interface.getPreferredOdomFrame(), StrEq("odom"));
  EXPECT_THAT(parameter_interface.getFramePrefix(), Eq(std::nullopt));
  EXPECT_THAT(parameter_interface.getTimeSyncTimeout(), Eq(std::chrono::seconds(5)));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetCamerasUsedDefaultWithArm) {
  // GIVEN we don't set the cameras_used parameter
  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and without custom gripperless firmware
  bool arm = true;
  bool gripperless = false;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN we get the default of all available cameras.
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_arm.value(),
              UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT, SpotCamera::LEFT, SpotCamera::RIGHT,
                                   SpotCamera::BACK, SpotCamera::HAND));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetCamerasUsedDefaultWithoutArm) {
  // Note: this cannot live in the same test as above, as the first call to getCamerasUsed will declare a parameter,
  // and here we want to explicitly test what happens with no parameter declared.

  // GIVEN we don't set the cameras_used parameter
  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot without an arm, and without custom gripperless firmware
  bool arm = false;
  bool gripperless = false;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN we get the default of all available cameras.
  const auto cameras_used_no_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_no_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_no_arm.value(), UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT,
                                                                SpotCamera::LEFT, SpotCamera::RIGHT, SpotCamera::BACK));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetCamerasUsedSubset) {
  // GIVEN we set cameras used to a subset of the available cameras
  const std::vector<std::string> cameras_used_parameter = {"frontleft", "frontright"};
  node_->declare_parameter("cameras_used", cameras_used_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and without custom gripperless firmware
  bool arm = true;
  bool gripperless = false;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN the returned values match the values we used when declaring the parameters
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_arm.value(), UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT));

  // GIVEN we are operating on a robot without an arm
  arm = false;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN the returned values match the values we used when declaring the parameters
  const auto cameras_used_no_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_no_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_no_arm.value(), UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetCamerasUsedSubsetWithHand) {
  // GIVEN we set cameras used to a subset of the available cameras including the hand camera
  const std::vector<std::string> cameras_used_parameter = {"frontleft", "frontright", "hand"};
  node_->declare_parameter("cameras_used", cameras_used_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and without custom gripperless firmware
  bool arm = true;
  bool gripperless = false;

  // WHEN we call the functions to get the config values from the parameter interface if the robot has an arm
  // THEN the returned values match the values we used when declaring the parameters
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_arm.value(),
              UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT, SpotCamera::HAND));

  // WHEN we call the functions to get the config values from the parameter interface if the robot does not have an arm
  // THEN this is an invalid choice of parameters.
  arm = false;
  const auto cameras_used_no_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_no_arm.has_value(), IsFalse());
  EXPECT_THAT(cameras_used_no_arm.error(), StrEq("Cannot add SpotCamera 'hand', the robot does not have an arm!"));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetCamerasUsedWithInvalidCamera) {
  // GIVEN we set cameras used to a subset of the available cameras, with an invalid camera
  const std::vector<std::string> cameras_used_parameter = {"frontleft", "frontright", "not_a_camera"};
  node_->declare_parameter("cameras_used", cameras_used_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and without custom gripperless firmware
  bool arm = true;
  bool gripperless = false;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN the result is invalid for robots with and without arms, as the camera "not_a_camera" does not exist on Spot.
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsFalse());
  EXPECT_THAT(cameras_used_arm.error(), StrEq("Cannot convert camera 'not_a_camera' to a SpotCamera."));
  arm = false;
  const auto cameras_used_no_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_no_arm.has_value(), IsFalse());
  EXPECT_THAT(cameras_used_no_arm.error(), StrEq("Cannot convert camera 'not_a_camera' to a SpotCamera."));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetDefaultCamerasUsedGripperless) {
  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and WITH custom gripperless firmware
  bool arm = true;
  bool gripperless = true;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN we get the default of all available cameras, excluding the hand!
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_arm.value(), UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT,
                                                             SpotCamera::LEFT, SpotCamera::RIGHT, SpotCamera::BACK));

  // WHEN gripperless is set to true on a robot without an arm
  // THEN we still get the default of all available cameras, excluding the hand
  arm = false;
  const auto cameras_used_no_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_no_arm.has_value(), IsTrue());
  EXPECT_THAT(cameras_used_no_arm.value(), UnorderedElementsAre(SpotCamera::FRONTLEFT, SpotCamera::FRONTRIGHT,
                                                                SpotCamera::LEFT, SpotCamera::RIGHT, SpotCamera::BACK));
}

TEST_F(RclcppParameterInterfaceEnvVarTest, GetSelectedCamerasUsedGripperless) {
  // GIVEN we set cameras used to a subset of the available cameras including the hand camera
  const std::vector<std::string> cameras_used_parameter = {"frontleft", "frontright", "hand"};
  node_->declare_parameter("cameras_used", cameras_used_parameter);

  // GIVEN we create a RclcppParameterInterface using the node
  RclcppParameterInterface parameter_interface{node_};

  // GIVEN we are operating on a robot with an arm, and WITH custom gripperless firmware
  bool arm = true;
  bool gripperless = true;

  // WHEN we call the functions to get the config values from the parameter interface
  // THEN this is an invalid choice of parameters, as the hand camera is not available on gripperless robots.
  const auto cameras_used_arm = parameter_interface.getCamerasUsed(arm, gripperless);
  EXPECT_THAT(cameras_used_arm.has_value(), IsFalse());
  EXPECT_THAT(cameras_used_arm.error(), StrEq("Cannot add SpotCamera 'hand', the robot is gripperless!"));
}

TEST_F(RclcppParameterInterfaceTest, GetFramePrefixFromNamespaceFallback) {
  // GIVEN we create rclcpp nodes with and without a specific namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  const auto namespaced_node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we create a RclcppParameterInterface using these nodes
  RclcppParameterInterface parameter_interface_a{node};
  RclcppParameterInterface parameter_interface_b{namespaced_node};

  // WHEN we call getFramePrefixWithDefaultFallback
  // THEN the parameter interface returns a frame prefix based on the nodes' namespaces
  const std::string expected_prefix = std::string(kNamespace) + kFramePrefixSeparator;
  EXPECT_THAT(parameter_interface_a.getFramePrefixWithDefaultFallback(), StrEq(""));
  EXPECT_THAT(parameter_interface_b.getFramePrefixWithDefaultFallback(), StrEq(expected_prefix));
}

TEST_F(RclcppParameterInterfaceTest, GetFramePrefixFromSpotNameFallback) {
  // GIVEN we create rclcpp nodes with and without a specific namespace
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  const auto namespaced_node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we set the spot name config parameter to an explicit value
  node->declare_parameter("spot_name", kSpotName);
  namespaced_node->declare_parameter("spot_name", kSpotName);

  // GIVEN we create a RclcppParameterInterface using these nodes
  RclcppParameterInterface parameter_interface_a{node};
  RclcppParameterInterface parameter_interface_b{namespaced_node};

  // WHEN we call getFramePrefixWithDefaultFallback
  // THEN the parameter interface returns a frame prefix based on the spot name in both cases
  const std::string expected_prefix = std::string(kSpotName) + kFramePrefixSeparator;
  EXPECT_THAT(parameter_interface_a.getFramePrefixWithDefaultFallback(), StrEq(expected_prefix));
  EXPECT_THAT(parameter_interface_b.getFramePrefixWithDefaultFallback(), StrEq(expected_prefix));
}

TEST_F(RclcppParameterInterfaceTest, GetFramePrefixFromExplicitParameter) {
  static constexpr auto verifyExpectedFramePrefix = [](std::shared_ptr<rclcpp::Node> node,
                                                       std::shared_ptr<rclcpp::Node> namespaced_node) -> void {
    // GIVEN we create a RclcppParameterInterface using these nodes
    RclcppParameterInterface parameter_interface_a{node};
    RclcppParameterInterface parameter_interface_b{namespaced_node};

    // WHEN we call getFramePrefixWithDefaultFallback
    // THEN the parameter interface returns the explicit frame prefix in both cases
    EXPECT_THAT(parameter_interface_a.getFramePrefixWithDefaultFallback(), StrEq(kFramePrefix));
    EXPECT_THAT(parameter_interface_b.getFramePrefixWithDefaultFallback(), StrEq(kFramePrefix));
  };

  // Set up first test scenario.
  // GIVEN we create rclcpp nodes with and without a specific namespace
  auto node = std::make_shared<rclcpp::Node>(kNodeName);
  auto namespaced_node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we only set the frame prefix config parameter to an explicit value, without any spot name
  node->declare_parameter("frame_prefix", kFramePrefix);
  namespaced_node->declare_parameter("frame_prefix", kFramePrefix);
  // Finish first test scenario.
  verifyExpectedFramePrefix(node, namespaced_node);

  // Set up second test scenario.
  // NOTE: We're creating new nodes for the second test scenario, since we can't undeclare statically typed parameters.
  node.reset();
  namespaced_node.reset();
  node = std::make_shared<rclcpp::Node>(kNodeName);
  namespaced_node = std::make_shared<rclcpp::Node>(kNodeName, kNamespace);
  // GIVEN we set both, the spot name and frame prefix, config parameters to explicit values
  node->declare_parameter("spot_name", kSpotName);
  namespaced_node->declare_parameter("spot_name", kSpotName);
  node->declare_parameter("frame_prefix", kFramePrefix);
  namespaced_node->declare_parameter("frame_prefix", kFramePrefix);
  // Finish second test scenario.
  verifyExpectedFramePrefix(node, namespaced_node);
}

TEST_F(RclcppParameterInterfaceTest, GetConfigWithInvalidOptions) {
  // GIVEN we create an rclcpp node and set config parameters to invalid values
  const auto node = std::make_shared<rclcpp::Node>(kNodeName);
  node->declare_parameter("preferred_odom_frame", "visionvision");
  node->declare_parameter("tf_root", "visionvision");

  // GIVEN we create a RclcppParameterInterface using this node
  RclcppParameterInterface parameter_interface{node};

  // WHEN we get the values from the parameter interface
  // THEN the parameter interface returns the default options
  EXPECT_THAT(parameter_interface.getPreferredOdomFrame(), StrEq("odom"));
  EXPECT_THAT(parameter_interface.getTFRoot(), StrEq("odom"));
}
}  // namespace spot_ros2::test
