// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <spot_driver/api/kinematic_api.hpp>
#include <spot_driver/api/lease_client_interface.hpp>
#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/api/state_client_interface.hpp>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/interfaces/image_client_interface.hpp>

#include <memory>
#include <optional>
#include <string>
#include <tl_expected/expected.hpp>

namespace spot_ros2::test {
class MockSpotApi : public SpotApi {
 public:
  MOCK_METHOD((tl::expected<void, std::string>), createRobot,
              (const std::string&, const std::string&, const std::optional<int>&), (override));
  MOCK_METHOD((tl::expected<void, std::string>), authenticate, (const std::string&, const std::string&), (override));
  MOCK_METHOD((tl::expected<bool, std::string>), hasArm, (), (const, override));
  MOCK_METHOD(std::shared_ptr<KinematicApi>, kinematicInterface, (), (const, override));
  MOCK_METHOD(std::shared_ptr<ImageClientInterface>, image_client_interface, (), (const, override));
  MOCK_METHOD(std::shared_ptr<StateClientInterface>, stateClientInterface, (), (const, override));
  MOCK_METHOD(std::shared_ptr<LeaseClientInterface>, leaseClientInterface, (), (const, override));
  MOCK_METHOD(std::shared_ptr<TimeSyncApi>, timeSyncInterface, (), (const, override));
  MOCK_METHOD(std::shared_ptr<WorldObjectClientInterface>, worldObjectClientInterface, (), (const, override));
};
}  // namespace spot_ros2::test
