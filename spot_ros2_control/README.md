# spot_ros2_control

This is a ROS 2 package designed to communicate with Spot's joint control API through ROS 2 control.
It uses the hardware interface provided in the [`spot_hardware_interface`](../spot_hardware_interface/) package to connect to the robot (and also supports a `mock` hardware interface that forwards commands directly to state).
By default, it loads a standard joint state broadcaster and forward position controller provided from `ros2_control`.
Alternative controllers from [`spot_controllers`](../spot_controllers/) that forwards position, velocity, effort, and gain commands at the same time are also available.

## On-robot

If your parameters for authenticating with your robot are set in the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot
```

Login information can also be set in a configuration file, the same as in the `spot_driver` launchfile.
For example, if you have a config file `spot_ros.yaml` containing the following information:
```
/**:
  ros__parameters:
    username: "username"
    password: "password"
    hostname: "00.00.00.00"
```
You can then run the launchfile with the following command:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot config_file:=path/to/spot_ros.yaml
```

If you wish to launch these nodes in a namespace, add the argument `spot_name:=<Robot Name>` (note that this will also prefix the joints).

This hardware interface will stream the joint angles of the robot at 333 Hz onto the topic `/<Robot Name>/low_level/joint_states`.

By default, the ros2 control stack will launch using `forward_position_controller`.
This uses a built-in ROS 2 controller for streaming joint level commands on the topic `/<Robot Name>/forward_position_controller/commands`.
This will forward position commands directly to the joint control API through the hardware interface.
The controller expects the command array to contain the list of positions to forward for each joint on the robot (12 elements for robots without an arm, and 19 for robots with an arm).

An alternative feed-forward controller provided by the [`spot_controllers`](../spot_controllers/) package can be used to specify the position, velocity, and effort of all joints at the same time.
To bring up this controller, add the launch argument `robot_controller:=forward_state_controller`.
Commands can then be sent on the topic `/<Robot Name>/forward_state_controller/commands`.
This controller expects the ordering of the command array to be `[<positions for each joint>, <velocities for each joint>, <efforts for each joint>]` (36 elements for robots without an arm, and 57 for robots with an arm).

Finally, the custom controller `spot_joint_controller` is also provided for the ability to stream position, velocity, effort, k_q_p, and k_qd_p for any subset of joints.
To enable this controller, add the launch argument `robot_controller:=spot_joint_controller`.
This controller accepts commands on the topic `/<Robot Name>/spot_joint_controller/joint_commands`.
Unlike the previous controllers, you do not need to set a value for every single joint, but instead can select the joints by name you wish to control in the `joint_commands` message -- all other joints that are not specified will be left unchanged.
More details on this controller can be found on the [`spot_controllers` README](../spot_controllers/README.md).

> [!CAUTION]
> When using these available controllers, there is no safety mechanism in place to ensure smooth motion.
Specifically, for the forward position and state controllers, the ordering of the command must match the ordering of the joints specified in the controller configuration file ([here for robots with an arm](config/spot_default_controllers_with_arm.yaml) or [here for robots without an arm](config/spot_default_controllers_without_arm.yaml)), and the robot can move in unpredictable and dangerous ways if this is not set correctly.
Make sure to keep a safe distance from the robot when working with these controllers and ensure the e-stop can easily be pressed if needed.

Additionally, the state publisher node, object synchronization node, and image publisher nodes from [`spot_driver`](../spot_driver/) will get launched by default when running on the robot to provide extra information such as TF, odometry, and camera feeds.
To turn off the image publishers (which can cause problems with bandwidth), add the launch argument `launch_image_publishers:=false`.

### Setting Gains
As mentioned above, joint level gains can be streamed to the robot while using `spot_joint_controller`.
If you are using a different controller, but still want to set the initial value of each of these gains, joint level gains can also be specified in the `config_file` under the parameter names `k_q_p` and `k_qd_p`.
If you do not specify these parameters, the default gains from the `spot-sdk` joint control examples are used during command streaming.
The example below shows a valid configuration for a robot with an arm as each list contains 19 elements.
More information on how these gains are used by Spot can be found [here](https://dev.bostondynamics.com/docs/concepts/joint_control/readme).
```
/**:
  ros__parameters:
    k_q_p: [624.0, 936.0, 286.0,  624.0, 936.0, 286.0, 624.0, 936.0, 286.0, 624.0, 936.0, 286.0, 1020.0, 255.0, 204.0, 102.0, 102.0, 102.0, 16.0]
    k_qd_p: [5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 5.20, 5.20, 2.04, 10.2, 15.3, 10.2, 2.04, 2.04, 2.04, 0.32]
```

## Mock

To use a mock hardware interface, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=mock
```

By default, this will load a robot with no arm. If you want your mock robot to have an arm, add the launch argument `mock_arm:=True`. 

## Examples

Examples are provided to replicate [these joint control examples](https://github.com/boston-dynamics/spot-cpp-sdk/tree/master/cpp/examples/joint_control) from Boston Dynamics.
They are designed to show how you can send commands to the robot using the built in `forward_position_controller`, and can be run in `mock` mode or on the real robot.

Run the following commands to test these examples after launching `spot_ros2_control.launch.py`. If your robot does not have an arm:
```bash
ros2 launch spot_ros2_control noarm_squat.launch.py
```
Or, if your robot does have an arm:
```bash
ros2 launch spot_ros2_control wiggle_arm.launch.py
```
Add the launch argument `spot_name:=<namespace>` if the ros2 control stack was launched in a namespace.

An alternate example using the `spot_joint_controller` is provided to demonstrate how to stream position and gains at the same time.
First launch `spot_ros2_control.launch.py` with the launch argument `robot_controller:=spot_joint_controller`.
Next, run
```bash
ros2 run spot_ros2_control set_grippper_gains
```
Include the argument `--robot <namespace>` if the ros2 control stack was launched in a namespace.
This demo will repeatedly open and close the gripper, and after each motion, will take user input on new k_q_p and k_qd_p values to use next.

## Additional Arguments
* `controllers_config`: If this argument is unset, a general purpose controller configuration will be loaded containing a forward position controller and a joint state publisher, that is filled appropriately based on whether or not the robot used (mock or real) has an arm. The forward state controller and spot joint controller are also specified here. If you wish to load different controllers, this can be set here.
* `robot_controller`: This is the name of the robot controller that will be started when the launchfile is called. The default is the simple forward position controller. The name must match a controller in the `controllers_config` file.
* `launch_rviz`: If you do not want rviz to be launched, add the argument `launch_rviz:=False`.
