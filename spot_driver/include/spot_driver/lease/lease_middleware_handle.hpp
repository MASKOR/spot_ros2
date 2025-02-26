// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <spot_driver/lease/lease_manager.hpp>

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

namespace spot_ros2::lease {

class LeaseMiddlewareHandle : public LeaseManager::MiddlewareHandle {
 public:
  explicit LeaseMiddlewareHandle(std::shared_ptr<rclcpp::Node> node);

  void createClaimLeasesService(
      const std::string& service_name,
      std::function<void(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>)> callback)
      override;

  void createAcquireLeaseService(
      const std::string& service_name,
      std::function<void(const std::shared_ptr<AcquireLease::Request>, std::shared_ptr<AcquireLease::Response>)>
          callback) override;

  void createReturnLeaseService(
      const std::string& service_name,
      std::function<void(const std::shared_ptr<ReturnLease::Request>, std::shared_ptr<ReturnLease::Response>)> callback)
      override;

  void createReleaseLeasesService(
      const std::string& service_name,
      std::function<void(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>)> callback)
      override;

  std::shared_ptr<LeaseManager::MiddlewareHandle::Bond> createBond(const std::string& node_name,
                                                                   std::function<void()> break_callback) override;

  void publishLeaseArray(std::unique_ptr<LeaseArray> message) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<LeaseArray>> lease_array_publisher_;
  std::shared_ptr<rclcpp::Service<Trigger>> claim_leases_service_;
  std::shared_ptr<rclcpp::Service<AcquireLease>> acquire_lease_service_;
  std::shared_ptr<rclcpp::Service<ReturnLease>> return_lease_service_;
  std::shared_ptr<rclcpp::Service<Trigger>> release_leases_service_;
};
}  // namespace spot_ros2::lease
