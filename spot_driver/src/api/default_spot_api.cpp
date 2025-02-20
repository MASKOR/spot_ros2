// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <bosdyn/client/gripper_camera_param/gripper_camera_param_client.h>
#include <bosdyn/client/world_objects/world_object_client.h>
#include <memory>
#include <spot_driver/api/default_image_client.hpp>
#include <spot_driver/api/default_kinematic_api.hpp>
#include <spot_driver/api/default_lease_client.hpp>
#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/api/default_state_client.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <tl_expected/expected.hpp>
#include "spot_driver/api/default_world_object_client.hpp"
#include "spot_driver/api/state_client_interface.hpp"

namespace spot_ros2 {

DefaultSpotApi::DefaultSpotApi(const std::string& sdk_client_name, const std::chrono::seconds timesync_timeout,
                               const std::optional<std::string>& certificate)
    : timesync_timeout_(timesync_timeout) {
  if (certificate.has_value()) {
    client_sdk_ = std::make_unique<::bosdyn::client::ClientSdk>();
    client_sdk_->SetClientName(sdk_client_name);
    if (const auto status = client_sdk_->LoadRobotCertFromFile(certificate.value()); !status) {
      throw std::runtime_error(status.message());
    }
    client_sdk_->Init();
  } else {
    client_sdk_ = ::bosdyn::client::CreateStandardSDK(sdk_client_name);
  }
}

tl::expected<void, std::string> DefaultSpotApi::createRobot(const std::string& robot_name,
                                                            const std::string& ip_address,
                                                            const std::optional<int>& port) {
  robot_name_ = robot_name;

  auto create_robot_result = client_sdk_->CreateRobot(ip_address, ::bosdyn::client::USE_PROXY);
  if (!create_robot_result.status) {
    return tl::make_unexpected("Received error result when creating SDK robot interface: " +
                               create_robot_result.status.DebugString());
  }

  robot_ = std::move(create_robot_result.response);

  if (port.has_value()) {
    robot_->UpdateSecureChannelPort(port.value());
  }

  return {};
}

tl::expected<void, std::string> DefaultSpotApi::authenticate(const std::string& username, const std::string& password) {
  const auto authenticate_result = robot_->Authenticate(username, password);
  if (!authenticate_result) {
    return tl::make_unexpected("Authentication with provided username and password did not succeed: " +
                               authenticate_result.DebugString());
  }
  // Start time synchronization between the robot and the client system.
  // This must be done only after a successful authentication.
  const auto start_time_sync_response = robot_->StartTimeSync();
  if (!start_time_sync_response) {
    return tl::make_unexpected("Failed to start time synchronization.");
  }

  const auto get_time_sync_thread_response = robot_->GetTimeSyncThread();
  if (!get_time_sync_thread_response) {
    return tl::make_unexpected("Failed to get the time synchronization thread.");
  }
  time_sync_api_ = std::make_shared<DefaultTimeSyncApi>(get_time_sync_thread_response.response, timesync_timeout_);

  // Image API.
  const auto image_client_result = robot_->EnsureServiceClient<::bosdyn::client::ImageClient>(
      ::bosdyn::client::ImageClient::GetDefaultServiceName());
  if (!image_client_result.status) {
    return tl::make_unexpected("Failed to create Image client.");
  }
  // TODO(jschornak-bdai): apply clock skew in the image publisher instead of in DefaultImageClient
  image_client_interface_ =
      std::make_shared<DefaultImageClient>(image_client_result.response, time_sync_api_, robot_name_);

  const auto robot_state_result = robot_->EnsureServiceClient<::bosdyn::client::RobotStateClient>(
      ::bosdyn::client::RobotStateClient::GetDefaultServiceName());
  if (!robot_state_result.status) {
    return tl::make_unexpected("Failed to get robot state service client.");
  }
  state_client_interface_ = std::make_shared<DefaultStateClient>(robot_state_result.response);

  // Lease API.
  const auto lease_client_result = robot_->EnsureServiceClient<::bosdyn::client::LeaseClient>(
      ::bosdyn::client::LeaseClient::GetDefaultServiceName());
  if (!lease_client_result.status) {
    return tl::make_unexpected("Failed to create Lease client.");
  }
  lease_client_interface_ = std::make_shared<DefaultLeaseClient>(lease_client_result.response);

  // Kinematic API.
  const auto kinematic_api_result = robot_->EnsureServiceClient<::bosdyn::client::InverseKinematicsClient>(
      ::bosdyn::client::InverseKinematicsClient::GetDefaultServiceName());
  if (!kinematic_api_result.status) {
    // Failure to create the kinematic interface is not an error state, since it does not exist in older versions of the
    // Spot firmware, so don't return here.
    kinematic_interface_ = nullptr;
  } else {
    // The kinematic interface is only available if the corresponding Spot API client was successfully created.
    kinematic_interface_ = std::make_shared<DefaultKinematicApi>(kinematic_api_result.response);
  }

  const auto world_object_client_result = robot_->EnsureServiceClient<::bosdyn::client::WorldObjectClient>(
      ::bosdyn::client::WorldObjectClient::GetDefaultServiceName());
  if (!world_object_client_result.status) {
    return tl::make_unexpected("Failed to create world object client: " +
                               world_object_client_result.status.DebugString());
  }
  if (world_object_client_result.response == nullptr) {
    return tl::make_unexpected("Failed to create world object client (nullptr): " +
                               world_object_client_result.status.DebugString());
  }
  world_object_client_interface_ = std::make_shared<DefaultWorldObjectClient>(world_object_client_result.response);

  return {};
}

tl::expected<bool, std::string> DefaultSpotApi::hasArm() const {
  // Determine if Spot has an arm by checking if the client for gripper camera parameters exists, since Spots without
  // arms do not have this client.
  const auto list_result = robot_->ListServices();
  if (!list_result.status) {
    return tl::make_unexpected("Failed to retrieve list of Spot services.");
  }

  const auto& services = list_result.response;

  return std::find_if(services.cbegin(), services.cend(), [](const ::bosdyn::api::ServiceEntry& entry) {
           return entry.name() == ::bosdyn::client::GripperCameraParamClient::GetDefaultServiceName();
         }) != services.cend();
}

std::shared_ptr<ImageClientInterface> DefaultSpotApi::image_client_interface() const {
  return image_client_interface_;
}

std::shared_ptr<StateClientInterface> DefaultSpotApi::stateClientInterface() const {
  return state_client_interface_;
}

std::shared_ptr<LeaseClientInterface> DefaultSpotApi::leaseClientInterface() const {
  return lease_client_interface_;
}

std::shared_ptr<KinematicApi> DefaultSpotApi::kinematicInterface() const {
  return kinematic_interface_;
}

std::shared_ptr<TimeSyncApi> DefaultSpotApi::timeSyncInterface() const {
  return time_sync_api_;
}

std::shared_ptr<WorldObjectClientInterface> DefaultSpotApi::worldObjectClientInterface() const {
  return world_object_client_interface_;
}

}  // namespace spot_ros2
