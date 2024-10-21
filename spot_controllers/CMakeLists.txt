# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

cmake_minimum_required(VERSION 3.22)

# This is here so we can use jthread from C++ 20
set(CMAKE_CXX_STANDARD 20)

project(spot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  forward_command_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
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

generate_parameter_library(
  forward_state_controller_parameters
  include/spot_controllers/forward_state_controller_parameters.yaml
)

# Add the hardware interface
add_library(
  spot_controllers
  SHARED
  src/forward_state_controller.cpp
)
target_compile_features(spot_controllers PUBLIC cxx_std_20)
target_include_directories(spot_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/spot_controllers>
)
target_link_libraries(
  spot_controllers PUBLIC 
  forward_state_controller_parameters forward_command_controller::forward_command_controller
)
ament_target_dependencies(
  spot_controllers PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SPOT_CONTROLLERS_BUILDING_DLL")

# Export controller plugin
pluginlib_export_plugin_description_file(controller_interface spot_controllers.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS spot_controllers forward_state_controller_parameters
  EXPORT export_spot_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_spot_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
