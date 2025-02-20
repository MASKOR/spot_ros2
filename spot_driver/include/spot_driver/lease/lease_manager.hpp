// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/api/lease_client_interface.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>

#include <spot_msgs/srv/acquire_lease.hpp>
#include <spot_msgs/srv/return_lease.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace spot_ros2::lease {

using spot_msgs::srv::AcquireLease;
using spot_msgs::srv::ReturnLease;

/**
 * Lease management for a ROS 2 graph.
 *
 * This manager acts as a proxy for leasing, allowing multiple
 * ROS 2 nodes to cooperate without losing ownership over robot
 * hardware.
 */
class LeaseManager {
 public:
  /**
   * This middleware handle is used to register services and instantiate
   * node bonds. In testing, it can be mocked to avoid using the ROS
   * infrastructure.
   */
  class MiddlewareHandle {
   public:
    class Bond {
     public:
      virtual ~Bond() = default;
    };

    virtual void createAcquireLeaseService(
        const std::string& service_name,
        std::function<void(const std::shared_ptr<AcquireLease::Request>, std::shared_ptr<AcquireLease::Response>)>
            callback) = 0;

    virtual void createReturnLeaseService(
        const std::string& service_name,
        std::function<void(const std::shared_ptr<ReturnLease::Request>, std::shared_ptr<ReturnLease::Response>)>
            callback) = 0;

    virtual std::shared_ptr<Bond> createBond(const std::string& node_name, std::function<void()> break_callback) = 0;

    virtual ~MiddlewareHandle() = default;
  };

  /**
   * Instantiate the lease manager.
   * @param lease_client The ROS node.
   * @param logger Logging interface.
   * @param middleware_handle
   */
  explicit LeaseManager(std::shared_ptr<LeaseClientInterface> lease_client, std::shared_ptr<LoggerInterfaceBase> logger,
                        std::unique_ptr<MiddlewareHandle> middleware_handle);

  /** Initialize the service. */
  void initialize();

  /**
   * Acquire lease on request.
   *
   * Typically used to implement the associated service.
   *
   * @param request The ROS request.
   * @param response A ROS response to be filled.
   */
  void acquireLease(const std::shared_ptr<AcquireLease::Request> request,
                    std::shared_ptr<AcquireLease::Response> response);

  /**
   * Return lease on request.
   *
   * Typically used to implement the associated service.
   *
   * @param request The ROS request.
   * @param response A ROS response to be filled.
   */
  void returnLease(const std::shared_ptr<ReturnLease::Request> request,
                   std::shared_ptr<ReturnLease::Response> response);

 private:
  /**
   * Handle lease retention failure.
   *
   * Typically hooked up to the underlying lease keepalive.
   *
   * @param result The lease use result returned on lease retention failure.
   */
  void onLeaseRetentionFailure(const bosdyn::api::LeaseUseResult& result);

  // An interface to the lease API.
  std::shared_ptr<LeaseClientInterface> lease_client_;

  // An interface to logging API.
  std::shared_ptr<LoggerInterfaceBase> logger_;

  // An interface to the middleware provider.
  std::unique_ptr<MiddlewareHandle> middleware_handle_;

  struct ManagedSublease {
    bosdyn::client::Lease lease;                   // Actual SDK sublease.
    std::shared_ptr<MiddlewareHandle::Bond> bond;  // ROS layer keepalive.
  };
  // Storage for active subleases (and associated keepalives).
  std::unordered_map<std::string, ManagedSublease> subleases_;

  // Synchronization primitive for sublease storage.
  std::recursive_mutex subleases_mutex_;
};
}  // namespace spot_ros2::lease
