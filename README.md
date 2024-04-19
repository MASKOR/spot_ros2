<p align="center">
  <img src="spot.png" width="350">
  <h1 align="center">Spot ROS 2 Driver</h1>
  <p align="center">
    <img src="https://img.shields.io/badge/python-3.8|3.9|3.10-blue"/>
    <a href="https://github.com/astral-sh/ruff">
      <img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json"/>
    </a>
    <a href="https://github.com/psf/black">
      <img src="https://img.shields.io/badge/code%20style-black-000000.svg"/>
    </a>
    <a href="https://github.com/bdaiinstitute/spot_ros2/actions/workflows/ci.yml">
      <img src="https://github.com/bdaiinstitute/spot_ros2/actions/workflows/ci.yml/badge.svg?branch=main"/>
    </a>
    <a href="https://coveralls.io/github/bdaiinstitute/spot_ros2?branch=main">
      <img src="https://coveralls.io/repos/github/bdaiinstitute/spot_ros2/badge.svg?branch=main"/>
    </a>
    <a href="LICENSE">
      <img src="https://img.shields.io/badge/license-MIT-purple"/>
    </a>
  </p>
</p>

# Overview
This is a ROS 2 package for Boston Dynamics' Spot. The package contains all necessary topics, services and actions to teleoperate or navigate Spot.
This package is derived from this [ROS 1 package](https://github.com/heuristicus/spot_ros). This package currently corresponds to version 4.0.0 of the [spot-sdk](https://github.com/boston-dynamics/spot-sdk/releases/tag/v4.0.0).

## Prerequisites
This package is tested for Ubuntu 22.04 and ROS 2 Humble, which can be installed following [this guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 

## Installation
In your ROS 2 workspace `src` directory, clone the repository:
```bash
git clone https://github.com/bdaiinstitute/spot_ros2.git
```
and initialize and install the submodules:
```bash
cd spot_ros2
git submodule init
git submodule update
```

Then run the install script to install the necessary Boston Dynamics and ROS dependencies. The install script takes the optional argument ```--arm64```; it otherwise defaults to an AMD64 install. Run the correct command based on your system:
```bash
cd <path to spot_ros2>
./install_spot_ros2.sh
or
./install_spot_ros2.sh --arm64
```
From here, build and source the ROS 2 workspace:
```
cd <ros2 ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-ignore proto2ros_tests
source install/local_setup.bash
```

We suggest ignoring the `proto2ros_tests` package in the build as it is not necessary for running the driver. If you choose to build it, you will see a number of error messages from testing the failure paths. 

# Spot ROS 2 Driver

The Spot driver contains all of the necessary topics, services, and actions for controlling Spot over ROS 2. To launch the driver, run:
```
ros2 launch spot_driver spot_driver.launch.py [config_file:=<path/to/config.yaml>] [spot_name:=<Robot Name>] [publish_point_clouds:=<True|False>] [launch_rviz:=<True|False>] 
```

## Configuration

The Spot login data hostname, username and password can be specified either as ROS parameters or as environment variables.  If using ROS parameters, see `spot_driver/config/spot_ros_example.yaml` for an example of what your file could look like.  If using environment variables, define `BOSDYN_CLIENT_USERNAME`, `BOSDYN_CLIENT_PASSWORD`, and `SPOT_IP`.

## Simple Robot Commands
Many simple robot commands can be called as services from the command line once the driver is running. For example:

* `ros2 service call /<Robot Name>/sit std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/stand std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/undock std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/power_off std_srvs/srv/Trigger`

If your Spot has an arm, some additional helpful services are exposed:
* `ros2 service call /<Robot Name>/stow_arm std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/ready_arm std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/open_gripper std_srvs/srv/Trigger`
* `ros2 service call /<Robot Name>/close_gripper std_srvs/srv/Trigger`

The full list of interfaces provided by the driver can be explored via `ros2 topic list`, `ros2 service list`, and `ros2 action list`. For more information about the custom message types used in this package, run `ros2 interface show <interface_type>`.

## Examples
See [`spot_examples`](spot_examples/) for some more complex examples of using the ROS 2 driver to control Spot, which typically use the action servers provided by the driver. 

## Images
By default, the driver will publish RGB images as well as depth maps from the `frontleft`, `frontright`, `left`, `right`, and `back` cameras on Spot (plus `hand` if your Spot has an arm). If your Spot has greyscale cameras, you will need to set `rgb_cameras: False` in your configuration YAML file, or you will not recieve any image data. 

By default, the driver does not publish point clouds. To enable this, launch the driver with `publish_point_clouds:=True`.

## Spot CAM
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


# Advanced Install

## Install spot_msgs as a deb package
`spot_msgs` are normally compiled as part of this repository.  If you would prefer to install them as a debian package, follow the steps below:
```bash
wget -q -O /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb https://github.com/bdaiinstitute/spot_ros2/releases/download/spot_msgs-v0.0-0/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
sudo dpkg -i /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
rm /tmp/ros-humble-spot-msgs_0.0.0-0jammy_amd64.deb
```

## Install bosdyn_msgs from source
The `bosdyn_msgs` package is installed as a debian package as part of the `install_spot_ros2` script because it's very large.  It can be checked out from source [here](https://github.com/bdaiinstitute/bosdyn_msgs) and then built as a normal ROS 2 package if that is preferred (compilation takes about 15 minutes).


# Help

If you encounter problems when using this repository, feel free to open an issue or PR.

## Verify Boston Dynamics API Installation
If you encounter `ModuleNotFoundErrors` with `bosdyn` packages upon running the driver, it is likely that the necessary Boston Dynamics API packages did not get installed with `install_spot_ros2.sh`. To check this, you can run the following command. Note that all versions should be `4.0.0`. 
```bash
$ pip list | grep bosdyn
bosdyn-api                           4.0.0
bosdyn-api-msgs                      4.0.0
bosdyn-auto-return-api-msgs          4.0.0
bosdyn-autowalk-api-msgs             4.0.0
bosdyn-choreography-client           4.0.0
bosdyn-choreography-protos           4.0.0
bosdyn-client                        4.0.0
bosdyn-core                          4.0.0
bosdyn-graph-nav-api-msgs            4.0.0
bosdyn-keepalive-api-msgs            4.0.0
bosdyn-log-status-api-msgs           4.0.0
bosdyn-metrics-logging-api-msgs      4.0.0
bosdyn-mission                       4.0.0
bosdyn-mission-api-msgs              4.0.0
bosdyn-msgs                          4.0.0
bosdyn-spot-api-msgs                 4.0.0
bosdyn-spot-cam-api-msgs             4.0.0
```
If these packages were not installed correctly on your system, you can try manually installing them following [Boston Dynamics' guide](https://dev.bostondynamics.com/docs/python/quickstart#install-spot-python-packages).

## Numpy & `spot_msgs`
If `spot_msgs` fails to build because it cannot locate numpy, [this issue](https://github.com/bdaiinstitute/spot_ros2/issues/96) has some solutions to this problem.

## Anaconda
`spot_ros2` does not currently support a conda install, but it is possible to run the driver in a conda environment with a little extra work -- [this issue](https://github.com/bdaiinstitute/spot_ros2/issues/330) covers some of the additional installs needed for this.

# License

MIT license - parts of the code developed specifically for ROS 2.
BSD3 license - parts of the code derived from the Clearpath Robotics ROS 1 driver.

# Contributing
To contribute, install `pre-commit` via pip, run `pre-commit install` and then run `pre-commit run --all-files` to 
verify that your code will pass inspection. 
```bash
git clone https://github.com/bdaiinstitute/spot_ros2.git
cd spot_ros2
pip3 install pre-commit
pre-commit install
pre-commit run --all-files
```

Now whenever you commit code to this repository, it will be checked against our `pre-commit` hooks. You can also run
`git commit --no-verify` if you wish to commit without checking against the hooks. 

## Contributors

This project is a collaboration between the [Mobile Autonomous Systems & Cognitive Robotics Institute](https://maskor.fh-aachen.de/en/) (MASKOR) at [FH Aachen](https://www.fh-aachen.de/en/) and the [Boston Dynamics AI Institute](https://theaiinstitute.com/).

MASKOR contributors:

* Maximillian Kirsch
* Shubham Pawar
* Christoph Gollok
* Stefan Schiffer
* Alexander Ferrein

Boston Dynamics AI Institute contributors:

* Jenny Barry
* Daniel Gonzalez
* Tao Pang
* David Surovik
* Jiuguang Wang
* David Watkins

[Linköping University](https://liu.se/en/organisation/liu/ida) contributors:

* Tommy Persson
