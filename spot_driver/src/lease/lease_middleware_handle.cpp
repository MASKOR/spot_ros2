// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/lease/lease_middleware_handle.hpp>

#include <bondcpp/bond.hpp>

namespace spot_ros2::lease {

LeaseMiddlewareHandle::LeaseMiddlewareHandle(std::shared_ptr<rclcpp::Node> node) : node_{node} {}

void LeaseMiddlewareHandle::createAcquireLeaseService(
    const std::string& service_name,
    std::function<void(const std::shared_ptr<AcquireLease::Request>, std::shared_ptr<AcquireLease::Response>)>
        callback) {
  acquire_lease_service_ = node_->create_service<AcquireLease>(service_name, callback);
}

void LeaseMiddlewareHandle::createReturnLeaseService(
    const std::string& service_name,
    std::function<void(const std::shared_ptr<ReturnLease::Request>, std::shared_ptr<ReturnLease::Response>)> callback) {
  return_lease_service_ = node_->create_service<ReturnLease>(service_name, callback);
}

namespace {

class BondHolder : public LeaseManager::MiddlewareHandle::Bond {
 public:
  explicit BondHolder(std::shared_ptr<rclcpp::Node> node, const std::string& id, std::function<void()> break_callback)
      : bond_("bonds", id, std::move(node), std::move(break_callback)) {}

  ~BondHolder() override {}

 private:
  bond::Bond bond_;
};

}  // namespace

std::shared_ptr<LeaseManager::MiddlewareHandle::Bond> LeaseMiddlewareHandle::createBond(
    const std::string& node_name, std::function<void()> break_callback) {
  return std::make_shared<BondHolder>(node_, node_name, std::move(break_callback));
}

}  // namespace spot_ros2::lease
