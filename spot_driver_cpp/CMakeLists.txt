cmake_minimum_required(VERSION 3.22)

list(APPEND CMAKE_PREFIX_PATH /opt/spot-cpp-sdk)

project(spot_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_ROS_DEPENDS
  cv_bridge
  geometry_msgs
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tl_expected
)

find_package(ament_cmake REQUIRED)
find_package(bosdyn REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_ROS_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Base library containing the SpotImagePublisherNode class
add_library(spot_image_publisher
  src/interfaces/rclcpp_logger_interface.cpp
  src/interfaces/rclcpp_parameter_interface.cpp
  src/interfaces/rclcpp_publisher_interface.cpp
  src/interfaces/rclcpp_tf_interface.cpp
  src/interfaces/rclcpp_wall_timer_interface.cpp
  src/interfaces/spot_interface.cpp
  src/spot_image_publisher.cpp
  src/spot_image_publisher_node.cpp
  src/spot_image_sources.cpp)
target_include_directories(spot_image_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_image_publisher PUBLIC bosdyn::bosdyn_client_static)
set_property(TARGET spot_image_publisher PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(spot_image_publisher PUBLIC ${THIS_PACKAGE_INCLUDE_ROS_DEPENDS})

# Create executable to allow running SpotImagePublisherNode directly as a ROS 2 node
add_executable(spot_image_publisher_node src/spot_image_publisher_node_main.cpp)
target_link_libraries(spot_image_publisher_node PUBLIC spot_image_publisher)
target_include_directories(spot_image_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Register a composable node to allow loading SpotImagePublisherNode in a component container
add_library(spot_image_publisher_component SHARED src/spot_image_publisher_component.cpp)
target_include_directories(spot_image_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_image_publisher_component PUBLIC spot_image_publisher)
ament_target_dependencies(spot_image_publisher PUBLIC rclcpp_components)

rclcpp_components_register_node(
  spot_image_publisher_component
  PLUGIN "spot_ros2::SpotImagePublisherNode"
  EXECUTABLE spot_image_publisher_node_component)

# Install Libraries
install(
  TARGETS
    spot_image_publisher
    spot_image_publisher_component
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Binaries
install(
  TARGETS 
    spot_image_publisher_node
    spot_image_publisher_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
