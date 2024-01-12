// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/api/default_kinematic_api.hpp>

namespace spot_ros2 {
DefaultKinematicApi::DefaultKinematicApi(bosdyn::client::InverseKinematicsClient* kinematicClient)
    : kinematicClient_{kinematicClient} {}

tl::expected<Result<InverseKinematicsResponse>, std::string> DefaultKinematicApi::getSolutions(
    InverseKinematicsRequest& request) {
  try {
    return kinematicClient_->InverseKinematics(request);
  } catch (const std::exception& ex) {
    return tl::make_unexpected("Failed to query the InverseKinematics service: " + std::string{ex.what()});
  }
}

}  // namespace spot_ros2
