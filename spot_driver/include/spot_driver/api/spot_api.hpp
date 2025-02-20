// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/kinematic_api.hpp>
#include <spot_driver/api/lease_client_interface.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/api/world_object_client_interface.hpp>
#include <spot_driver/interfaces/image_client_interface.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <optional>
#include <string>

namespace spot_ros2 {

class SpotApi {
 public:
  // SpotApi is move-only
  SpotApi() = default;
  SpotApi(SpotApi&& other) = default;
  SpotApi(const SpotApi&) = delete;
  SpotApi& operator=(SpotApi&& other) = default;
  SpotApi& operator=(const SpotApi&) = delete;

  virtual ~SpotApi() = default;

  virtual tl::expected<void, std::string> createRobot(const std::string& robot_name, const std::string& ip_address,
                                                      const std::optional<int>& port = std::nullopt) = 0;
  virtual tl::expected<void, std::string> authenticate(const std::string& username, const std::string& password) = 0;
  virtual tl::expected<bool, std::string> hasArm() const = 0;
  /**
   * @brief Get a shared_ptr to the Spot API's inverse kinematics interface, if it could be created.
   * @details Spots with firmware older than 3.3.0 cannot create the inverse kinematics client, so it is not guaranteed
   * to exist.
   * @return A shared_ptr to the inverse kinematics interface. If this interface is not available, returns a nullptr.
   */
  virtual std::shared_ptr<KinematicApi> kinematicInterface() const = 0;
  virtual std::shared_ptr<ImageClientInterface> image_client_interface() const = 0;

  /**
   * @brief Get a StateClientInterface that communicates with Spot's robot state server.
   * @return A shared_ptr to an instance of StateClientInterface which is owned by this object.
   */
  virtual std::shared_ptr<StateClientInterface> stateClientInterface() const = 0;
  [[nodiscard]] virtual std::shared_ptr<LeaseClientInterface> leaseClientInterface() const = 0;
  virtual std::shared_ptr<TimeSyncApi> timeSyncInterface() const = 0;
  [[nodiscard]] virtual std::shared_ptr<WorldObjectClientInterface> worldObjectClientInterface() const = 0;
};
}  // namespace spot_ros2
