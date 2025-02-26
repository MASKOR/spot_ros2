// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include <spot_driver/api/lease_client_interface.hpp>

namespace spot_ros2::test {
class MockLeaseClient : public LeaseClientInterface {
 public:
  MOCK_METHOD((tl::expected<::bosdyn::client::Lease, std::string>), acquireLease,
              (const std::string&, LeaseUseCallback), (override));

  MOCK_METHOD((tl::expected<::bosdyn::client::Lease, std::string>), takeLease, (const std::string&, LeaseUseCallback),
              (override));

  MOCK_METHOD((tl::expected<bool, std::string>), returnLease, (const ::bosdyn::client::Lease&), (override));

  MOCK_METHOD((tl::expected<std::vector<::bosdyn::api::LeaseResource>, std::string>), listLeases, (), (override));

  MOCK_METHOD((std::shared_ptr<::bosdyn::client::LeaseWallet>), getLeaseWallet, (), (const, override));

  MOCK_METHOD((const ::bosdyn::client::ResourceHierarchy&), getResourceHierarchy, (), (const, override));
};
}  // namespace spot_ros2::test
