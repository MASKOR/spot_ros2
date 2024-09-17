# spot_ros2_control

This is a ROS 2 package designed to communicate with Spot's low level API through ROS 2 control. It sets up a hardware interface that can be either `mock` or connected to the robot using the [Spot C++ SDK](https://github.com/boston-dynamics/spot-cpp-sdk) (currently in progress). By default, it loads a standard joint state broadcaster and forward position controller provided from `ros2_control`.

## On-robot

If your parameters for authenticating with your robot are set in the environment variables `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=robot
```

Login information can also be set in a configuration file, the same as in the `spot_driver` launchfile. For example, if you have a config file `spot_ros.yaml` containing the following information:
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

This hardware interface can stream the joint angles of the robot using the low level API at ~333 Hz. 
Both state streaming and command streaming are implemented. 

## Mock

To use a mock hardware interface, run:

```bash
ros2 launch spot_ros2_control spot_ros2_control.launch.py hardware_interface:=mock
```

By default, this will load a robot with no arm. If you want your mock robot to have an arm, add the launch argument `mock_has_arm:=True`. 

## Examples

Examples are provided to replicate [these joint control examples](https://github.com/boston-dynamics/spot-cpp-sdk/tree/master/cpp/examples/joint_control) from Boston Dynamics. They are currently only supported in `mock` mode, but are designed to show how you can send commands to the robot using the built in forward position controller.

Run the following commands to test these examples after launching `spot_ros2_control.launch.py`:
```bash
ros2 launch spot_ros2_control noarm_squat.launch.py
```
```bash
ros2 launch spot_ros2_control wiggle_arm.launch.py
```

## Additional Arguments
* `controllers_config`: If this argument is unset, a general purpose controller configuration will be loaded containing a forward position controller and a joint state publisher, that is filled appropriately based on whether or not the robot used (mock or real) has an arm. If you wish to load a different controller, this can be set here.
* `robot_controller`: This is the name of the robot controller that will be started when the launchfile is called. The default is the simple forward position controller. The name must match a controller in the `controllers_config` file.
* `launch_rviz`: If you do not want rviz to be launched, add the argument `launch_rviz:=False`.
