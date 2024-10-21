# spot_controllers

This is a ROS 2 package that provides custom ROS 2 controllers that can be used with [spot_ros2_control](../spot_ros2_control/).

Currently, this package consists of a single generic controller: `spot_controllers/ForwardStateController`. This controller allows you to forward a set of commands over a set of interfaces. It is used with `spot_ros2_control` to forwad commands for position, velocity, and effort for all joints at the same time. 

Example configurations for setting up this controller can be found in [`spot_ros2_control/config`](../spot_ros2_control/config/).
