// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
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

namespace spot_hardware_interface {

/// @brief Number of joints we expect if the robot has no arm
inline constexpr int kNjointsNoArm = 12;

/// @brief Number of joints we expect if the robot has an arm
inline constexpr int kNjointsArm = 19;

// Gain values https://github.com/boston-dynamics/spot-cpp-sdk/blob/master/cpp/examples/joint_control/constants.hpp
// This will be handled via a parameter in the future so there is the option to change them, for now they are hardcoded

/// @brief Default Kp gains for robot without an arm
inline constexpr float kDefaultKpNoArm[] = {624.0, 936.0, 286.0, 624.0, 936.0, 286.0,
                                            624.0, 936.0, 286.0, 624.0, 936.0, 286.0};

/// @brief Default Kd gains for robot without an arm
inline constexpr float kDefaultKdNoArm[] = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04};

/// @brief Default Kp gains for robot with an arm
inline constexpr float kDefaultKpArm[] = {624.0, 936.0, 286.0,  624.0, 936.0, 286.0, 624.0, 936.0, 286.0, 624.0,
                                          936.0, 286.0, 1020.0, 255.0, 204.0, 102.0, 102.0, 102.0, 16.0};

/// @brief Default Kd gains for robot with an arm
inline constexpr float kDefaultKdArm[] = {5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20,
                                          5.20, 2.04, 10.2, 15.3, 10.2, 2.04, 2.04, 2.04, 0.32};

}  // namespace spot_hardware_interface
