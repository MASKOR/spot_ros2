// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/api/default_lease_client.hpp>

#include <chrono>

namespace spot_ros2 {

DefaultLeaseClient::DefaultLeaseClient(bosdyn::client::LeaseClient* lease_client) : lease_client_{lease_client} {}

tl::expected<::bosdyn::client::Lease, std::string> DefaultLeaseClient::acquireLease(
    const std::string& resource_name, LeaseUseCallback retention_failure_callback) {
  try {
    auto result = lease_client_->AcquireLease(resource_name);
    if (!result) {
      return tl::make_unexpected(result.status.Chain("Could not acquire lease").message());
    }
    auto callback = [this, callback = std::move(retention_failure_callback)](
                        const ::bosdyn::client::Result<::bosdyn::api::RetainLeaseResponse>& result,
                        ::bosdyn::client::LeaseKeepAlive* lease_keep_alive) {
      callback(result.response.lease_use_result());
      lease_keep_alive->StopKeepAliveThread();
    };
    constexpr auto kKeepAlivePeriod = std::chrono::seconds(1);
    auto keepalive = std::make_unique<::bosdyn::client::LeaseKeepAlive>(
        lease_client_, lease_client_->GetLeaseWallet(), resource_name, kKeepAlivePeriod, std::move(callback));
    std::lock_guard<std::mutex> lock(keepalives_mutex_);
    keepalives_.emplace(std::make_pair(resource_name, std::move(keepalive)));
    return ::bosdyn::client::Lease(result.response.lease());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to acquire lease: " + std::string{ex.what()});
  }
}

tl::expected<bosdyn::client::Lease, std::string> DefaultLeaseClient::takeLease(
    const std::string& resource_name, LeaseUseCallback retention_failure_callback) {
  try {
    auto result = lease_client_->TakeLease(resource_name);
    if (!result) {
      return tl::make_unexpected(result.status.Chain("Could not acquire lease").message());
    }
    constexpr auto kKeepAlivePeriod = std::chrono::seconds(1);
    auto callback = [this, callback = std::move(retention_failure_callback)](
                        const ::bosdyn::client::Result<::bosdyn::api::RetainLeaseResponse>& result,
                        ::bosdyn::client::LeaseKeepAlive* lease_keep_alive) {
      callback(result.response.lease_use_result());
      lease_keep_alive->StopKeepAliveThread();
    };
    auto keepalive = std::make_unique<::bosdyn::client::LeaseKeepAlive>(
        lease_client_, lease_client_->GetLeaseWallet(), resource_name, kKeepAlivePeriod, std::move(callback));
    std::lock_guard<std::mutex> lock(keepalives_mutex_);
    keepalives_.emplace(std::make_pair(resource_name, std::move(keepalive)));
    return ::bosdyn::client::Lease(result.response.lease());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to take lease: " + std::string{ex.what()});
  }
}

tl::expected<bool, std::string> DefaultLeaseClient::returnLease(const ::bosdyn::client::Lease& lease) {
  try {
    auto request = ::bosdyn::api::ReturnLeaseRequest();
    *request.mutable_lease() = lease.GetProto();
    auto result = lease_client_->ReturnLease(request);
    if (!result) {
      return tl::make_unexpected(result.status.Chain("Could not return lease").message());
    }
    std::lock_guard<std::mutex> lock(keepalives_mutex_);
    keepalives_.erase(lease.GetResource());
    return true;
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to take lease: " + std::string{ex.what()});
  }
}

tl::expected<std::vector<::bosdyn::api::LeaseResource>, std::string> DefaultLeaseClient::listLeases() {
  try {
    auto result = lease_client_->ListLeases();
    if (!result) {
      return tl::make_unexpected(result.status.Chain("Could not list leases").message());
    }
    return std::vector(result.response.resources().begin(), result.response.resources().end());
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to take lease: " + std::string{ex.what()});
  }
}

std::shared_ptr<::bosdyn::client::LeaseWallet> DefaultLeaseClient::getLeaseWallet() const {
  return lease_client_->GetLeaseWallet();
}

const ::bosdyn::client::ResourceHierarchy& DefaultLeaseClient::getResourceHierarchy() const {
  return ::bosdyn::client::DefaultResourceHierarchy(::bosdyn::client::LeaseHierarchyRequirements::ARM_AND_GRIPPER);
}

}  // namespace spot_ros2
