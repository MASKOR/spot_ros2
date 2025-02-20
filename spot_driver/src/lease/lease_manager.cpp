// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/lease/lease_manager.hpp>

#include <functional>
#include <unordered_set>

#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/lease/lease_middleware_handle.hpp>

#include <bosdyn_api_msgs/conversions.hpp>

namespace {
constexpr auto kAcquireLeaseServiceName = "acquire_lease";
constexpr auto kReturnLeaseServiceName = "return_lease";
}  // namespace

namespace spot_ros2::lease {
LeaseManager::LeaseManager(std::shared_ptr<LeaseClientInterface> lease_client,
                           std::shared_ptr<LoggerInterfaceBase> logger,
                           std::unique_ptr<MiddlewareHandle> middleware_handle)
    : lease_client_{std::move(lease_client)},
      logger_{std::move(logger)},
      middleware_handle_{std::move(middleware_handle)} {}

void LeaseManager::initialize() {
  middleware_handle_->createAcquireLeaseService(
      kAcquireLeaseServiceName,
      [this](const std::shared_ptr<AcquireLease::Request> request, std::shared_ptr<AcquireLease::Response> response) {
        this->acquireLease(request, response);
      });
  middleware_handle_->createReturnLeaseService(
      kReturnLeaseServiceName,
      [this](const std::shared_ptr<ReturnLease::Request> request, std::shared_ptr<ReturnLease::Response> response) {
        this->returnLease(request, response);
      });
}

void LeaseManager::onLeaseRetentionFailure(const bosdyn::api::LeaseUseResult& result) {
  std::lock_guard<std::recursive_mutex> lock(subleases_mutex_);
  auto lease_wallet = lease_client_->getLeaseWallet();
  const google::protobuf::EnumDescriptor* descriptor = bosdyn::api::LeaseUseResult::Status_descriptor();
  const std::string& status = descriptor->FindValueByNumber(result.status())->name();
  const std::string& resource_name = result.attempted_lease().resource();
  logger_->logInfo(resource_name + " lease was revoked: " + status);
  lease_wallet->RemoveLease(resource_name);
  logger_->logInfo("Revoking " + resource_name + " subleases");
  subleases_.erase(resource_name);
}

void LeaseManager::acquireLease(const std::shared_ptr<AcquireLease::Request> request,
                                std::shared_ptr<AcquireLease::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(subleases_mutex_);

  const auto& root_resource_hierarchy = lease_client_->getResourceHierarchy();
  if (!root_resource_hierarchy.HasResource(request->resource_name)) {
    response->message = request->resource_name + " is not a known resource";
    response->success = false;
    return;
  }

  auto lease_wallet = lease_client_->getLeaseWallet();

  std::unordered_set<std::string> potential_collisions{root_resource_hierarchy.Resource()};
  for (const auto& [name, hierarchy] : root_resource_hierarchy) {
    if (hierarchy.HasResource(request->resource_name)) {
      potential_collisions.insert(name);
    }
  }
  const auto& resource_hierarchy = root_resource_hierarchy.GetHierarchy(request->resource_name);
  for (const auto& [name, _] : resource_hierarchy) {
    potential_collisions.insert(name);
  }
  const auto& leaf_resources = resource_hierarchy.LeafResources();
  potential_collisions.insert(leaf_resources.begin(), leaf_resources.end());

  bool must_take = false;
  for (const auto& resource_name : potential_collisions) {
    if (subleases_.count(resource_name) > 0) {
      const auto& sublease_proto = subleases_[resource_name].lease.GetProto();
      const std::string& client_name = *sublease_proto.client_names().rbegin();
      response->message = resource_name + " is busy, subleased by " + client_name;
      response->success = false;
      return;
    }
    if (auto existing_lease = lease_wallet->GetLease(resource_name); existing_lease) {
      must_take = true;
    }
  }

  auto lease = lease_wallet->AdvanceLease(request->resource_name);
  if (!lease) {
    using std::placeholders::_1;
    auto callback = std::bind(&LeaseManager::onLeaseRetentionFailure, this, _1);
    if (must_take) {
      auto result = lease_client_->takeLease(request->resource_name, std::move(callback));
      if (!result) {
        response->message = "failed to take lease: " + result.error();
        response->success = false;
        return;
      }
      logger_->logInfo(request->resource_name + " lease was taken");
    } else {
      auto result = lease_client_->acquireLease(request->resource_name, std::move(callback));
      if (!result) {
        response->message = "failed to acquire lease: " + result.error();
        response->success = false;
        return;
      }
      logger_->logInfo(request->resource_name + " lease was acquired");
    }
    lease = lease_wallet->GetLease(request->resource_name);
  }

  auto sublease = lease.move().CreateSublease(request->client_name);
  bosdyn_api_msgs::conversions::Convert(sublease.GetProto(), &response->lease);

  auto bond = middleware_handle_->createBond(request->client_name, [this, sublease]() {
    std::lock_guard<std::recursive_mutex> lock(subleases_mutex_);
    const std::string& resource_name = sublease.GetResource();
    if (subleases_.erase(resource_name) > 0) {
      const auto& sublease_proto = sublease.GetProto();
      const std::string& client_name = *sublease_proto.client_names().rbegin();
      logger_->logError(client_name + " bond broken");

      using std::placeholders::_1;
      auto callback = std::bind(&LeaseManager::onLeaseRetentionFailure, this, _1);
      auto result = lease_client_->takeLease(resource_name, std::move(callback));
      if (!result) {
        logger_->logError("Failed to (re)take " + resource_name + " sublease: " + result.error());
      }
    }
    logger_->logInfo(resource_name + " sublease revoked");
  });
  subleases_[request->resource_name] = ManagedSublease{std::move(sublease), std::move(bond)};
  logger_->logInfo(request->resource_name + " sublease was acquired");
  response->success = true;
}

void LeaseManager::returnLease(const std::shared_ptr<ReturnLease::Request> request,
                               std::shared_ptr<ReturnLease::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(subleases_mutex_);

  bosdyn::api::Lease sublease_proto;
  bosdyn_api_msgs::conversions::Convert(request->lease, &sublease_proto);
  auto sublease = bosdyn::client::Lease(sublease_proto);

  const std::string& resource_name = sublease.GetResource();
  if (subleases_.count(resource_name) == 0) {
    response->message = resource_name + " is not subleased, cannot return";
    response->success = false;
    return;
  }
  using CompareResult = bosdyn::client::Lease::CompareResult;
  if (subleases_[resource_name].lease.Compare(sublease) != CompareResult::SAME) {
    response->message = resource_name + " sublease does not match the known lease";
    response->success = false;
    return;
  }
  subleases_.erase(resource_name);
  response->success = true;

  logger_->logInfo(resource_name + " sublease was returned");
}

}  // namespace spot_ros2::lease
