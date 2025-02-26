// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver/lease/lease_manager.hpp>

#include <chrono>
#include <functional>
#include <unordered_set>

#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/lease/lease_middleware_handle.hpp>

#include <bosdyn_api_msgs/conversions.hpp>

namespace {
constexpr auto kClaimLeasesServiceName = "claim_leases";
constexpr auto kAcquireLeaseServiceName = "acquire_lease";
constexpr auto kReturnLeaseServiceName = "return_lease";
constexpr auto kReleaseLeasesServiceName = "release_leases";
}  // namespace

namespace spot_ros2::lease {
LeaseManager::LeaseManager(std::shared_ptr<LeaseClientInterface> lease_client,
                           std::unique_ptr<LoggerInterfaceBase> logger_interface,
                           std::unique_ptr<TimerInterfaceBase> timer_interface,
                           std::unique_ptr<MiddlewareHandle> middleware_handle, std::optional<double> lease_check_rate)
    : lease_client_{std::move(lease_client)},
      logger_interface_{std::move(logger_interface)},
      timer_interface_{std::move(timer_interface)},
      middleware_handle_{std::move(middleware_handle)},
      lease_check_rate_{lease_check_rate} {}

void LeaseManager::initialize() {
  middleware_handle_->createClaimLeasesService(
      kClaimLeasesServiceName,
      [this](const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        this->claimLeases(request, response);
      });

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

  middleware_handle_->createReleaseLeasesService(
      kReleaseLeasesServiceName,
      [this](const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response) {
        this->releaseLeases(request, response);
      });

  if (lease_check_rate_.has_value()) {
    auto callback = std::bind(&LeaseManager::checkLeasingStatus, this);
    timer_interface_->setTimer(std::chrono::duration<double>(1.0 / lease_check_rate_.value()), std::move(callback));
  }
}

void LeaseManager::checkLeasingStatus() {
  auto result = lease_client_->listLeases();
  if (!result) {
    logger_interface_->logError(result.error());
    return;
  }
  auto message = std::make_unique<LeaseArray>();
  for (const auto& lease_resource_proto : result.value()) {
    auto& resource_message = message->resources.emplace_back();
    resource_message.resource = lease_resource_proto.resource();
    const auto& lease_proto = lease_resource_proto.lease();
    resource_message.lease.resource = lease_proto.resource();
    resource_message.lease.epoch = lease_proto.epoch();
    resource_message.lease.sequence.assign(lease_proto.sequence().begin(), lease_proto.sequence().end());
    const auto& lease_owner_proto = lease_resource_proto.lease_owner();
    resource_message.lease_owner.client_name = lease_owner_proto.client_name();
    resource_message.lease_owner.user_name = lease_owner_proto.user_name();
  }
  middleware_handle_->publishLeaseArray(std::move(message));
}

void LeaseManager::claimLeases(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  auto lease_wallet = lease_client_->getLeaseWallet();
  if (const auto resources = lease_wallet->GetAllOwnedResources(); !resources.empty()) {
    response->message = "leases acquired already for: ";
    for (size_t i = 0; i < resources.size() - 1; ++i) {
      response->message += resources[i] + ", ";
    }
    response->message += resources.back();
    response->success = true;
    return;
  }

  const auto& root_resource_hierarchy = lease_client_->getResourceHierarchy();
  const auto& root_resource_name = root_resource_hierarchy.Resource();

  using std::placeholders::_1;
  auto callback = std::bind(&LeaseManager::onLeaseRetentionFailure, this, _1);
  auto result = lease_client_->acquireLease(root_resource_name, std::move(callback));
  if (!result) {
    response->message = "failed to claim leases: " + result.error();
    response->success = false;
    return;
  }

  logger_interface_->logInfo("Leases claimed, " + root_resource_name + " lease owned");
  response->message = "Leases claimed, " + root_resource_name + " lease owned";
  response->success = true;
}

void LeaseManager::releaseLeases(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  auto lease_wallet = lease_client_->getLeaseWallet();
  for (const auto& lease : lease_wallet->GetAllLeases()) {
    auto result = lease_client_->returnLease(lease);
    if (!result) {
      logger_interface_->logError(result.error());
    }
    lease_wallet->RemoveLease(lease.GetResource());
  }
  subleases_.clear();

  logger_interface_->logInfo("Leases returned, subleases dropped");
  response->message = "Leases returned, subleases dropped";
  response->success = true;
}

void LeaseManager::onLeaseRetentionFailure(const bosdyn::api::LeaseUseResult& result) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto lease_wallet = lease_client_->getLeaseWallet();
  const google::protobuf::EnumDescriptor* descriptor = bosdyn::api::LeaseUseResult::Status_descriptor();
  const std::string& status = descriptor->FindValueByNumber(result.status())->name();
  const std::string& resource_name = result.attempted_lease().resource();
  logger_interface_->logInfo(resource_name + " lease was revoked: " + status);
  lease_wallet->RemoveLease(resource_name);
  logger_interface_->logInfo("Revoking " + resource_name + " subleases");
  subleases_.erase(resource_name);
}

void LeaseManager::acquireLease(const std::shared_ptr<AcquireLease::Request> request,
                                std::shared_ptr<AcquireLease::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

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
      logger_interface_->logInfo(request->resource_name + " lease was taken");
    } else {
      auto result = lease_client_->acquireLease(request->resource_name, std::move(callback));
      if (!result) {
        response->message = "failed to acquire lease: " + result.error();
        response->success = false;
        return;
      }
      logger_interface_->logInfo(request->resource_name + " lease was acquired");
    }
    lease = lease_wallet->GetLease(request->resource_name);
  }

  auto sublease = lease.move().CreateSublease(request->client_name);
  bosdyn_api_msgs::conversions::Convert(sublease.GetProto(), &response->lease);

  auto bond = middleware_handle_->createBond(request->client_name, [this, sublease]() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const std::string& resource_name = sublease.GetResource();
    if (subleases_.erase(resource_name) > 0) {
      const auto& sublease_proto = sublease.GetProto();
      const std::string& client_name = *sublease_proto.client_names().rbegin();
      logger_interface_->logError(client_name + " bond broken");

      using std::placeholders::_1;
      auto callback = std::bind(&LeaseManager::onLeaseRetentionFailure, this, _1);
      auto result = lease_client_->takeLease(resource_name, std::move(callback));
      if (!result) {
        logger_interface_->logError("Failed to (re)take " + resource_name + " sublease: " + result.error());
      }
    }
    logger_interface_->logInfo(resource_name + " sublease revoked");
  });
  subleases_[request->resource_name] = ManagedSublease{std::move(sublease), std::move(bond)};

  logger_interface_->logInfo(request->resource_name + " sublease was acquired");
  response->message = request->resource_name + " sublease was acquired";
  response->success = true;
}

void LeaseManager::returnLease(const std::shared_ptr<ReturnLease::Request> request,
                               std::shared_ptr<ReturnLease::Response> response) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

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

  logger_interface_->logInfo(resource_name + " sublease was returned");
  response->message = resource_name + " sublease was returned";
  response->success = true;
}

}  // namespace spot_ros2::lease
