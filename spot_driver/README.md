# spot_driver

The Spot driver contains all of the necessary topics, services, and actions for controlling Spot over ROS 2.
To launch the driver, run the following command, with the appropriate launch arguments and/or config file that are discussed below.
```
ros2 launch spot_driver spot_driver.launch.py [config_file:=<path/to/config.yaml>] [spot_name:=<Spot Name>] [tf_prefix:=<TF Frame Prefix>] [launch_rviz:=<True|False>] [launch_image_publishers:=<True|False>] [publish_point_clouds:=<True|False>] [uncompress_images:=<True|False>] [publish_compressed_images:=<True|False>] [stitch_front_images:=<True|False>]
```

## Configuration
The Spot login data hostname, username and password can be specified either as ROS parameters or as environment variables.
If using ROS parameters, see [`spot_driver/config/spot_ros_example.yaml`](spot_driver/config/spot_ros_example.yaml) for an example of what your file could look like, and pass this to the driver as a launch argument with `config_file:=path/to/config.yaml`.
If using environment variables, define `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`.

## Namespacing
By default, the driver is launched in the global namespace.
To avoid this, it is recommended to either launch the driver with the launch argument `spot_name:=<Spot Name>` or update the `spot_name` parameter in your config file (a specified launch argument will override the config file parameter).
This will place all of the nodes, topics, services, and actions provided by the driver in the `<Spot Name>` namespace.

By default, it will also prefix all of the TF frames and joints of the robot with `<Spot Name>`.
If you want to change this behavior and instead use a custom prefix `<TF Frame Prefix>` for all frames in the TF tree, either launch the driver with the launch argument `tf_prefix:=<TF Frame Prefix>` or update the `frame_prefix` parameter in your config file (a specified launch argument will override the config file parameter).
If you use the config file parameter `frame_prefix`, you can disable prefixing altogether by setting it to an empty string.

## Frames
Background information about Spot's frames from Boston Dynamics can be found [here](https://dev.bostondynamics.com/docs/concepts/geometry_and_frames). 
By default, the Spot driver will place the "odom" frame as the root of the TF tree.
This can be changed by setting the `tf_root` parameter in your config file to either "vision" or "body" (value must be given without a prefix).
The Spot driver will also publish odometry topics with respect to the "odom" frame by default.
If you wish to change this to "vision", update the `preferred_odom_frame` parameter in your config file (value must be given without a prefix).

## Simple Robot Commands
Many simple robot commands can be called as services from the command line once the driver is running. For example:

* `ros2 service call /<Robot Name>/sit std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/stand std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/undock std_srvs/srv/Trigger`

If your Spot has an arm, some additional helpful services are exposed:
* `ros2 service call /<Robot Name>/arm_stow std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/arm_unstow std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/arm_carry std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/open_gripper std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/close_gripper std_srvs/srv/Trigger`

## More Complex Robot Commands
The full list of interfaces provided by the driver can be explored via `ros2 topic list`, `ros2 service list`, and `ros2 action list`. 
For more information about the custom message types used in this package, run `ros2 interface show <interface_type>`. 
More details can also be found on the [`spot_ros2` wiki](https://github.com/bdaiinstitute/spot_ros2/wiki/Spot-Driver-Available-Interfaces). 


## Images
Perception data from Spot is handled through the `spot_image_publishers.launch.py` launchfile, which is launched by default from the driver.
If you want to only view images from Spot, without bringing up any of the nodes to control the robot, you can also choose to run this launchfile independently.

By default, the driver will publish RGB images as well as depth maps from the `frontleft`, `frontright`, `left`, `right`, and `back` cameras on Spot (plus `hand` if your Spot has an arm).
You can customize the cameras that are streamed from by adding the `cameras_used` parameter to your config yaml. (For example, to stream from only the front left and front right cameras, you can add `cameras_used: ["frontleft", "frontright"]`).
Additionally, if your Spot has greyscale cameras, you will need to set `rgb_cameras: False` in your configuration YAML file, or you will not receive any image data.

By default, the driver does not publish point clouds.
To enable this, launch the driver with `publish_point_clouds:=True`.

The driver can publish both compressed images (under `/<Robot Name>/camera/<camera location>/compressed`) and uncompressed images (under `/<Robot Name>/camera/<camera location>/image`).
By default, it will only publish the uncompressed images.
You can turn (un)compressed images on/off by launching the driver with the flags `uncompress_images:=<True|False>` and `publish_compressed_images:=<True|False>`.

The driver also has the option to publish a stitched image created from Spot's front left and front right cameras (similar to what is seen on the tablet).
If you wish to enable this, launch the driver with `stitch_front_images:=True`, and the image will be published under `/<Robot Name>/camera/frontmiddle_virtual/image`.
In order to receive meaningful stitched images, you will have to specify the parameters `virtual_camera_intrinsics`, `virtual_camera_projection_plane`, `virtual_camera_plane_distance`, and `stitched_image_row_padding` (see [`spot_driver/config/spot_ros_example.yaml`](spot_driver/config/spot_ros_example.yaml) for some default values). 

> **_NOTE:_**  
If your image publishing rate is very slow, you can try 
> - connecting to your robot via ethernet cable 
> - exporting a custom DDS profile we have provided by running the following in the same terminal your driver will run in, or adding to your `.bashrc`:
> ```
> export=FASTRTPS_DEFAULT_PROFILES_FILE=<path_to_file>/custom_dds_profile.xml
> ```

## Calibration
A calibration procedure for the hand camera is provided by this package. For more information on how to run this, refer to [EyeInHandCalibration.md](EyeInHandCalibration.md)


<details>
<summary><h2>Spot CAM</h2></summary>
<br>

Due to known issues with the Spot CAM, it is disabled by default. To enable publishing and usage over the driver, add the following command in your configuration YAML file:
    `initialize_spot_cam: True`

The Spot CAM payload has known issues with the SSL certification process in https. If you get the following errors:

```
non-existing PPS 0 referenced
decode_slice_header error
no frame!
```

Then you want to log into the Spot CAM over the browser. In your browser, type in:

    https://<ip_address_of_spot>:<sdp_port>/h264.sdp.html

The default port for SDP is 31102 for the Spot CAM. Once inside, you will be prompted to log in using your username and password. Do so and the WebRTC frames should begin to properly stream.

</details>


## Examples
For some examples of using the Spot ROS 2 driver, check out [`spot_examples`](../spot_examples/).
