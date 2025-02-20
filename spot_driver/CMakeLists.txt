cmake_minimum_required(VERSION 3.22)

project(spot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

set(THIS_PACKAGE_INCLUDE_ROS_DEPENDS
  bondcpp
  bosdyn_api_msgs
  bosdyn_cmake_module
  bosdyn_spot_api_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_eigen
  tf2_ros
  tl_expected
  spot_msgs
  nav_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_ROS_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(bosdyn REQUIRED)
find_package(OpenCV 4 REQUIRED)

###
# Spot API
###

add_library(spot_api
  src/api/default_kinematic_api.cpp
  src/api/default_image_client.cpp
  src/api/default_lease_client.cpp
  src/api/default_spot_api.cpp
  src/api/default_state_client.cpp
  src/api/default_time_sync_api.cpp
  src/api/default_world_object_client.cpp
  src/api/spot_image_sources.cpp
  src/conversions/common_conversions.cpp
  src/conversions/decompress_images.cpp
  src/conversions/geometry.cpp
  src/conversions/kinematic_conversions.cpp
  src/conversions/robot_state.cpp
  src/conversions/time.cpp
  src/images/spot_image_publisher.cpp
  src/images/images_middleware_handle.cpp
  src/images/spot_image_publisher_node.cpp
  src/interfaces/rclcpp_clock_interface.cpp
  src/interfaces/rclcpp_logger_interface.cpp
  src/interfaces/rclcpp_node_interface.cpp
  src/interfaces/rclcpp_parameter_interface.cpp
  src/interfaces/rclcpp_tf_broadcaster_interface.cpp
  src/interfaces/rclcpp_tf_listener_interface.cpp
  src/interfaces/rclcpp_wall_timer_interface.cpp
  src/lease/lease_manager.cpp
  src/lease/lease_manager_node.cpp
  src/lease/lease_middleware_handle.cpp
  src/kinematic/kinematic_node.cpp
  src/kinematic/kinematic_service.cpp
  src/kinematic/kinematic_middleware_handle.cpp
  src/object_sync/object_synchronizer.cpp
  src/object_sync/object_synchronizer_node.cpp
  src/robot_state/state_middleware_handle.cpp
  src/robot_state/state_publisher.cpp
  src/robot_state/state_publisher_node.cpp
)

target_include_directories(spot_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(spot_api PUBLIC bosdyn::bosdyn_client)
set_property(TARGET spot_api PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(spot_api PUBLIC ${THIS_PACKAGE_INCLUDE_ROS_DEPENDS})

###
# Spot image publisher
###

# Create executable to allow running SpotImagePublisherNode directly as a ROS 2 node
add_executable(spot_image_publisher_node src/images/spot_image_publisher_node_main.cpp)
target_include_directories(spot_image_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_image_publisher_node PUBLIC spot_api)

# Register a composable node to allow loading SpotImagePublisherNode in a component container
add_library(spot_image_publisher_component SHARED src/images/spot_image_publisher_component.cpp)
target_include_directories(spot_image_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_image_publisher_component PUBLIC spot_api)

rclcpp_components_register_node(
  spot_image_publisher_component
  PLUGIN "spot_ros2::images::SpotImagePublisherNode"
  EXECUTABLE spot_image_publisher_node_component)

###
# Spot state publisher
###

# Create executable to allow running StatePublisherNode directly as a ROS 2 node
add_executable(state_publisher_node src/robot_state/state_publisher_node_main.cpp)
target_link_libraries(state_publisher_node PUBLIC spot_api)
target_include_directories(state_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Register a composable node to allow loading StatePublisherNode in a component container
add_library(state_publisher_component SHARED src/robot_state/state_publisher_component.cpp)
target_include_directories(state_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(state_publisher_component PUBLIC spot_api)
ament_target_dependencies(state_publisher_component PUBLIC rclcpp_components)

rclcpp_components_register_node(
  state_publisher_component
  PLUGIN "spot_ros2::StatePublisherNode"
  EXECUTABLE state_publisher_node_component)

###
# Spot lease manager
###

# Create executable to allow running LeaseManagerNode directly as a ROS 2 node
add_executable(lease_manager_node src/lease/lease_manager_node_main.cpp)
target_link_libraries(lease_manager_node PUBLIC spot_api)
target_include_directories(lease_manager_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Register a composable node to allow loading StatePublisherNode in a component container
add_library(lease_manager_component SHARED src/lease/lease_manager_component.cpp)
target_include_directories(lease_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lease_manager_component PUBLIC spot_api)
ament_target_dependencies(lease_manager_component PUBLIC rclcpp_components)

rclcpp_components_register_node(
  lease_manager_component
  PLUGIN "spot_ros2::LeaseManagerNode"
  EXECUTABLE lease_manager_node_component)

###
# Spot IK
###

# Create executable to allow running KinematicNode directly as a ROS 2 node.
add_executable(spot_inverse_kinematics_node src/kinematic/kinematic_node_main.cpp)
target_include_directories(spot_inverse_kinematics_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_inverse_kinematics_node PUBLIC spot_api)

# Register a composable node to allow loading KinematicNode in a component container
add_library(spot_inverse_kinematics_component SHARED src/kinematic/kinematic_component.cpp)
target_include_directories(spot_inverse_kinematics_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(spot_inverse_kinematics_component PUBLIC spot_api)

###
# Object Synchronization
###

add_executable(object_synchronizer_node src/object_sync/object_synchronizer_node_main.cpp)
target_include_directories(object_synchronizer_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_synchronizer_node PUBLIC spot_api)

rclcpp_components_register_node(
  spot_inverse_kinematics_component
  PLUGIN "spot_ros2::kinematic::KinematicNode"
  EXECUTABLE spot_inverse_kinematics_node_component)

add_library(image_stitcher
  src/image_stitcher/image_stitcher.cpp
  src/image_stitcher/image_stitcher_node.cpp)
target_include_directories(image_stitcher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(image_stitcher PUBLIC spot_api)
set_property(TARGET image_stitcher PROPERTY POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(image_stitcher PUBLIC ${THIS_PACKAGE_INCLUDE_ROS_DEPENDS})
  
add_executable(image_stitcher_node src/image_stitcher/image_stitcher_node_main.cpp)
target_include_directories(image_stitcher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(image_stitcher_node PUBLIC image_stitcher)

ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
  spot_driver/spot_ros2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spot_ros2
  )
install(
  PROGRAMS
  spot_driver/calibrated_reregistered_hand_camera_depth_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  )
install(
  PROGRAMS
  spot_driver/spot_alerts.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spot_alerts
  )
# Install Libraries
install(
  TARGETS
    image_stitcher
    spot_api
    spot_image_publisher_component
    spot_inverse_kinematics_component
    state_publisher_component
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install Executables
install(
  TARGETS 
    image_stitcher_node
    object_synchronizer_node
    spot_image_publisher_node
    spot_image_publisher_node_component
    spot_inverse_kinematics_node
    spot_inverse_kinematics_node_component
    state_publisher_node
    state_publisher_node_component
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/ DESTINATION include)

# Install launch and configurations
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
