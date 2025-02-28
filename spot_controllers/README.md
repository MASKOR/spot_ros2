# spot_controllers

This is a ROS 2 package that provides custom ROS 2 controllers that can be used with [spot_ros2_control](../spot_ros2_control/).

This package contains two controllers: `spot_controllers/ForwardStateController` and `spot_controllers/SpotJointController`.
Example configurations for setting up this controllers can be found in [`spot_ros2_control/config`](../spot_ros2_control/config/).

## `ForwardStateController`

This is a generic controller allows you to directly forward a set of commands over a set of interfaces.
It builds on top of ROS 2 control's [`forward_command_controller`](https://github.com/ros-controls/ros2_controllers/tree/master/forward_command_controller).
It is used with `spot_ros2_control` to forwad commands for position, velocity, and effort for all joints at the same time. 


## `SpotJointController`

This is a streaming controller specific to Spot's hardware interface, used to directly forward any subset of position, velocity, load, k_q_p, and k_qd_p over any subset of Spot's joints.
It receives commands of type `spot_msgs/msg/JointCommand`, which is almost the same as a `sensor_msgs/msg/JointState` message but with extra fields for Spot's `k_q_p` and `k_qd_p` command interfaces.
This controller claims all command interfaces of the robot on activation, but your `JointCommands` do not have to contain commands for all interfaces -- they can just contain a subset.
The `name` field in this topic should contain the names of the joints you wish to control, and for each entry in `name`, you can fill one or more of the elements in `postition`, `velocity`, `effort`, `k_q_p`, or `k_qd_p`.
Any interfaces that are not specified in this message will be left unchanged at their last commanded value.

For example, to send one position command to one joint, the joint command would be formed such as:
```
name: ["joint_1"]
postion: [1.0]
velocity: []
effort: []
k_q_p: []
k_qd_p: []
```
Or to control position, velocity, and k_qd_p of 3 different joints:
```
name: ["joint_1", "joint_2", "joint_3"]
postion: [1.0, 2.0, 3.0]
velocity: []
effort: [4.0, 5.0, 6.0]
k_q_p: []
k_qd_p: [7.0, 8.0, 9.0]
```
