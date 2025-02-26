// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/client/lease/lease.h>
#include <bosdyn/client/lease/lease_client.h>
#include <bosdyn/client/lease/lease_keepalive.h>
#include <bosdyn/client/lease/lease_wallet.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <spot_driver/api/lease_client_interface.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace spot_ros2 {
/**
 * @brief Implements LeaseClientInterface.
 */
class DefaultLeaseClient : public LeaseClientInterface {
 public:
  explicit DefaultLeaseClient(::bosdyn::client::LeaseClient* lease_client);

  [[nodiscard]] tl::expected<::bosdyn::client::Lease, std::string> acquireLease(
      const std::string& resource_name, LeaseUseCallback retention_failure_callback) override;

  [[nodiscard]] tl::expected<::bosdyn::client::Lease, std::string> takeLease(
      const std::string& resource_name, LeaseUseCallback retention_failure_callback) override;

  [[nodiscard]] tl::expected<bool, std::string> returnLease(const ::bosdyn::client::Lease& lease) override;

  [[nodiscard]] tl::expected<std::vector<::bosdyn::api::LeaseResource>, std::string> listLeases() override;

  [[nodiscard]] std::shared_ptr<::bosdyn::client::LeaseWallet> getLeaseWallet() const override;

  [[nodiscard]] const ::bosdyn::client::ResourceHierarchy& getResourceHierarchy() const override;

 private:
  ::bosdyn::client::LeaseClient* lease_client_;
  std::unordered_map<std::string, std::unique_ptr<::bosdyn::client::LeaseKeepAlive>> keepalives_;
  std::mutex keepalives_mutex_;
};
}  // namespace spot_ros2
