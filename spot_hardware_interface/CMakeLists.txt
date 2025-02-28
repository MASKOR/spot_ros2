# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

cmake_minimum_required(VERSION 3.22)

# This is here so we can use jthread from C++ 20
set(CMAKE_CXX_STANDARD 20)

project(spot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  bondcpp
  hardware_interface
  bosdyn_cmake_module
  pluginlib
  rclcpp
  rclcpp_lifecycle
  spot_msgs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(bosdyn REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the hardware interface
add_library(
  spot_hardware_interface
  SHARED
  src/spot_hardware_interface.cpp
  src/spot_leasing_interface.cpp
)
target_compile_features(spot_hardware_interface PUBLIC cxx_std_20)
target_include_directories(spot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_hardware_interface PUBLIC bosdyn::bosdyn_client)
ament_target_dependencies(
  spot_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SPOT_HARDWARE_INTERFACE_BUILDING_DLL")

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface spot_hardware_interface.xml)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS spot_hardware_interface
  EXPORT export_spot_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_spot_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} bosdyn)

ament_package()
