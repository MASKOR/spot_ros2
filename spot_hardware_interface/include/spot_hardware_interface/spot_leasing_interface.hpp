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

#pragma once

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <bondcpp/bond.hpp>

#include <bosdyn/client/lease/lease.h>
#include <bosdyn/client/lease/lease_client.h>
#include <bosdyn/client/lease/lease_wallet.h>
#include <bosdyn/client/robot/robot.h>

#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>

#include <spot_msgs/srv/acquire_lease.hpp>
#include <spot_msgs/srv/return_lease.hpp>

#include <tl_expected/expected.hpp>

namespace spot_hardware_interface {

class LeasingInterface {
 public:
  virtual ~LeasingInterface() = default;

  virtual tl::expected<::bosdyn::client::Lease, std::string> AcquireLease(const std::string& resource) = 0;

  virtual tl::expected<::bosdyn::client::Lease, std::string> ReturnLease(const std::string& resource) = 0;
};

class DirectLeasingInterface : public LeasingInterface {
 public:
  explicit DirectLeasingInterface(::bosdyn::client::Robot* robot);

  tl::expected<::bosdyn::client::Lease, std::string> AcquireLease(const std::string& resource_name) override;

  tl::expected<::bosdyn::client::Lease, std::string> ReturnLease(const std::string& resource_name) override;

 private:
  ::bosdyn::client::Robot* robot_;
  ::bosdyn::client::LeaseClient* lease_client_;
};

class ProxiedLeasingInterface : public LeasingInterface {
 public:
  explicit ProxiedLeasingInterface(::bosdyn::client::Robot* robot);

  ~ProxiedLeasingInterface() override;

  tl::expected<::bosdyn::client::Lease, std::string> AcquireLease(const std::string& resource_name) override;

  tl::expected<::bosdyn::client::Lease, std::string> ReturnLease(const std::string& resource_name) override;

 private:
  std::shared_ptr<::bosdyn::client::LeaseWallet> lease_wallet_;

  std::shared_ptr<rclcpp::Node> foreground_node_;
  std::shared_ptr<rclcpp::Client<spot_msgs::srv::AcquireLease>> acquire_lease_client_;
  std::shared_ptr<rclcpp::Client<spot_msgs::srv::ReturnLease>> return_lease_client_;

  std::jthread background_loop_;
  std::shared_ptr<rclcpp::Node> background_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> background_executor_;

  std::unordered_map<std::string, std::unique_ptr<bond::Bond>> keepalive_bonds_;
};

}  // namespace spot_hardware_interface
