// Copyright (c) 2025 Boston Dynamics AI Institute LLC.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "spot_hardware_interface/spot_leasing_interface.hpp"

#include <bosdyn/api/lease.pb.h>
#include <bosdyn_api_msgs/conversions.hpp>

namespace spot_hardware_interface {

DirectLeasingInterface::DirectLeasingInterface(::bosdyn::client::Robot* robot) : robot_(robot) {}

tl::expected<::bosdyn::client::Lease, std::string> DirectLeasingInterface::AcquireLease(
    const std::string& resource_name) {
  if (!lease_client_) {
    ::bosdyn::client::Result<::bosdyn::client::LeaseClient*> result =
        robot_->EnsureServiceClient<::bosdyn::client::LeaseClient>();
    if (!result) {
      return tl::make_unexpected(result.status.Chain("Could not create lease client").message());
    }
    lease_client_ = result.move();
  }
  // NOTE(mhidalgo-bdai): ROS 2 control does not manage Ctrl + C properly in Humble,
  // so here we take the lease instead of acquiring it, as it won't return it properly
  // on process interruption.
  auto result = lease_client_->TakeLease(resource_name);
  if (!result) {
    return tl::make_unexpected(result.status.Chain("Could not create acquire lease").message());
  }
  return ::bosdyn::client::Lease(result.move().lease());
}

tl::expected<::bosdyn::client::Lease, std::string> DirectLeasingInterface::ReturnLease(
    const std::string& resource_name) {
  if (!lease_client_) {
    return tl::make_unexpected("No lease client, did you acquire first?");
  }
  auto lease_wallet = robot_->GetWallet();
  auto result = lease_wallet->GetOwnedLease(resource_name);
  if (!result) {
    return tl::make_unexpected(result.status.Chain("No lease owned for " + resource_name).message());
  }
  ::bosdyn::client::Lease lease = result.move();
  bosdyn::api::ReturnLeaseRequest request;
  request.mutable_lease()->CopyFrom(lease.GetProto());
  auto response = lease_client_->ReturnLease(request);
  if (!response) {
    return tl::make_unexpected(response.status.Chain("Could not return lease").message());
  }
  return lease;
}

ProxiedLeasingInterface::ProxiedLeasingInterface(::bosdyn::client::Robot* robot) : lease_wallet_(robot->GetWallet()) {
  foreground_node_ = std::make_shared<rclcpp::Node>("_foreground_" + lease_wallet_->GetClientName() + "_node");
  acquire_lease_client_ = foreground_node_->create_client<spot_msgs::srv::AcquireLease>("acquire_lease");
  return_lease_client_ = foreground_node_->create_client<spot_msgs::srv::ReturnLease>("return_lease");
  background_node_ = std::make_shared<rclcpp::Node>("_background_" + lease_wallet_->GetClientName() + "_node");
  background_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  background_executor_->add_node(foreground_node_);
  background_executor_->add_node(background_node_);
  background_loop_ = std::jthread([this]() {
    background_executor_->spin();
  });
}

ProxiedLeasingInterface::~ProxiedLeasingInterface() {
  background_executor_->cancel();
}

tl::expected<::bosdyn::client::Lease, std::string> ProxiedLeasingInterface::AcquireLease(
    const std::string& resource_name) {
  auto request = std::make_shared<spot_msgs::srv::AcquireLease::Request>();
  request->resource_name = resource_name;
  request->client_name = lease_wallet_->GetClientName();
  auto future = acquire_lease_client_->async_send_request(request);
  auto outcome = rclcpp::spin_until_future_complete(foreground_node_, future, std::chrono::seconds(5));
  if (outcome == rclcpp::FutureReturnCode::TIMEOUT) {
    acquire_lease_client_->remove_pending_request(future);
    return tl::make_unexpected("Timed out trying to acquire the lease");
  }
  if (outcome == rclcpp::FutureReturnCode::INTERRUPTED) {
    acquire_lease_client_->remove_pending_request(future);
    return tl::make_unexpected("Lease acquisition got interrupted");
  }
  auto response = future.get();
  if (!response->success) {
    return tl::make_unexpected("Could not acquire lease: " + response->message);
  }
  ::bosdyn::api::Lease lease_proto;
  bosdyn_api_msgs::conversions::Convert(response->lease, &lease_proto);
  auto lease = ::bosdyn::client::Lease(lease_proto);

  using SubLease = ::bosdyn::client::LeaseWallet::SubLease;
  lease_wallet_->AddLease(lease, SubLease::kNoSubLease);

  keepalive_bonds_[resource_name] =
      std::make_unique<bond::Bond>("bonds", lease_wallet_->GetClientName(), background_node_),
  keepalive_bonds_[resource_name]->start();

  return lease;
}

tl::expected<::bosdyn::client::Lease, std::string> ProxiedLeasingInterface::ReturnLease(
    const std::string& resource_name) {
  auto result = lease_wallet_->GetOwnedLease(resource_name);
  if (!result) {
    return tl::make_unexpected(result.status.Chain("No lease owned for " + resource_name).message());
  }
  ::bosdyn::client::Lease lease = result.move();
  auto request = std::make_shared<spot_msgs::srv::ReturnLease::Request>();
  bosdyn_api_msgs::conversions::Convert(lease.GetProto(), &request->lease);
  auto future = return_lease_client_->async_send_request(request);
  auto outcome = rclcpp::spin_until_future_complete(foreground_node_, future, std::chrono::seconds(5));
  if (outcome == rclcpp::FutureReturnCode::TIMEOUT) {
    return_lease_client_->remove_pending_request(future);
    return tl::make_unexpected("Timed out trying to return the lease");
  }
  if (outcome == rclcpp::FutureReturnCode::INTERRUPTED) {
    return_lease_client_->remove_pending_request(future);
    return tl::make_unexpected("Lease return got interrupted");
  }
  auto response = future.get();
  if (!response->success) {
    return tl::make_unexpected(response->message);
  }
  lease_wallet_->RemoveLease(resource_name);
  keepalive_bonds_.erase(resource_name);
  return lease;
}

}  // namespace spot_hardware_interface
