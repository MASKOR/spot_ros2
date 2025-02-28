# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

cmake_minimum_required(VERSION 3.22)

# This is here so we can use jthread from C++ 20
set(CMAKE_CXX_STANDARD 20)

project(spot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  spot_hardware_interface
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the hardware interface
add_library(
  spot_ros2_control
  SHARED
  src/spot_joint_map.cpp
)
target_compile_features(spot_ros2_control PUBLIC cxx_std_20)
target_include_directories(spot_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  spot_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Add example nodes and install them
add_executable(noarm_squat examples/noarm_squat.cpp)
target_link_libraries(noarm_squat spot_ros2_control)

add_executable(wiggle_arm examples/wiggle_arm.cpp)
target_link_libraries(wiggle_arm spot_ros2_control)

add_executable(joint_command_passthrough examples/joint_command_passthrough.cpp)
target_link_libraries(joint_command_passthrough spot_ros2_control)

install(TARGETS noarm_squat wiggle_arm joint_command_passthrough
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
  examples/set_gripper_gains.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_gripper_gains
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS spot_ros2_control
  EXPORT export_spot_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_spot_ros2_control HAS_LIBRARY_TARGET)

ament_package()
