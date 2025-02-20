// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/spot_api.hpp>

#include <bosdyn/client/robot/robot.h>
#include <bosdyn/client/sdk/client_sdk.h>

#include <spot_driver/api/lease_client_interface.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/api/world_object_client_interface.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace spot_ros2 {

class DefaultSpotApi : public SpotApi {
 public:
  explicit DefaultSpotApi(const std::string& sdk_client_name, const std::chrono::seconds timesync_timeout,
                          const std::optional<std::string>& certificate = std::nullopt);

  [[nodiscard]] tl::expected<void, std::string> createRobot(const std::string& robot_name,
                                                            const std::string& ip_address,
                                                            const std::optional<int>& port = std::nullopt) override;
  [[nodiscard]] tl::expected<void, std::string> authenticate(const std::string& username,
                                                             const std::string& password) override;
  [[nodiscard]] tl::expected<bool, std::string> hasArm() const override;
  [[nodiscard]] std::shared_ptr<KinematicApi> kinematicInterface() const override;
  [[nodiscard]] std::shared_ptr<ImageClientInterface> image_client_interface() const override;
  [[nodiscard]] std::shared_ptr<StateClientInterface> stateClientInterface() const override;
  [[nodiscard]] std::shared_ptr<LeaseClientInterface> leaseClientInterface() const override;
  [[nodiscard]] std::shared_ptr<TimeSyncApi> timeSyncInterface() const override;
  [[nodiscard]] std::shared_ptr<WorldObjectClientInterface> worldObjectClientInterface() const override;

 private:
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;
  std::shared_ptr<KinematicApi> kinematic_interface_;
  std::shared_ptr<ImageClientInterface> image_client_interface_;
  std::shared_ptr<StateClientInterface> state_client_interface_;
  std::shared_ptr<LeaseClientInterface> lease_client_interface_;
  std::shared_ptr<TimeSyncApi> time_sync_api_;
  std::shared_ptr<WorldObjectClientInterface> world_object_client_interface_;
  std::string robot_name_;
  const std::chrono::seconds timesync_timeout_;
};
}  // namespace spot_ros2
