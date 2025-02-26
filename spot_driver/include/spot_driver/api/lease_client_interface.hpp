// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <tl_expected/expected.hpp>

#include <bosdyn/api/lease.pb.h>
#include <bosdyn/client/lease/lease.h>
#include <bosdyn/client/lease/lease_wallet.h>

namespace spot_ros2 {
/**
 * @brief Interface class to interact with Spot SDK's lease client.
 */
class LeaseClientInterface {
 public:
  using LeaseUseCallback = std::function<void(const bosdyn::api::LeaseUseResult&)>;

  // LeaseClientInterface is move-only
  LeaseClientInterface() = default;
  LeaseClientInterface(LeaseClientInterface&& other) = default;
  LeaseClientInterface(const LeaseClientInterface&) = delete;
  LeaseClientInterface& operator=(LeaseClientInterface&& other) = default;
  LeaseClientInterface& operator=(const LeaseClientInterface&) = delete;

  virtual ~LeaseClientInterface() = default;

  /**
   * @brief Acquire the lease for a given resource.
   * @param resource_name Name of the resource.
   * @param retention_failure_callback Callback on lease retention failure, invoked by the underlying lease keepalive.
   * @return Returns an expected bearing the Lease if the lease was acquired. Otherwise, it will carry an error message
   * describing the failure.
   */
  virtual tl::expected<::bosdyn::client::Lease, std::string> acquireLease(
      const std::string& resource_name, LeaseUseCallback retention_failure_callback) = 0;

  /**
   * @brief Take (forcefully) the lease for a given resource.
   * @param resource_name Name of the resource.
   * @param retention_failure_callback Callback on lease retention failure, invoked by the underlying lease keepalive.
   * @return Returns an expected bearing the Lease if the lease was taken. Otherwise, it will carry an error message
   * describing the failure.
   */
  virtual tl::expected<::bosdyn::client::Lease, std::string> takeLease(const std::string& resource_name,
                                                                       LeaseUseCallback retention_failure_callback) = 0;

  /**
   * @brief Return a given lease.
   * @param lease Lease to be returned.
   * @return Returns an expected bearing true if the lease was returned.
   * Otherwise, it will carry an error message describing the failure.
   */
  virtual tl::expected<bool, std::string> returnLease(const ::bosdyn::client::Lease& lease) = 0;

  /**
   * @brief Return all active leases.
   * @return Returns an expected bearing a collection of leased resources.
   * Otherwise, it will carry an error message describing the failure.
   */
  virtual tl::expected<std::vector<::bosdyn::api::LeaseResource>, std::string> listLeases() = 0;

  /**
   * @return The underlying lease wallet.
   */
  virtual std::shared_ptr<::bosdyn::client::LeaseWallet> getLeaseWallet() const = 0;

  /**
   * @return The resource hierarchy describing the robot hardware.
   */
  virtual const ::bosdyn::client::ResourceHierarchy& getResourceHierarchy() const = 0;
};
}  // namespace spot_ros2
