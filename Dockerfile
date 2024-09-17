FROM osrf/ros:humble-desktop-full

# Install dependencies (taken from devcontainer dockerfile)
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    curl \
    wget \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev && \
    rm -rf /var/lib/apt/lists/*

# Create ROS workspace
WORKDIR /ros_ws/src

# Clone driver code
RUN git clone https://github.com/bdaiinstitute/spot_ros2.git .
RUN git submodule update --init --recursive

# Run install script
RUN /ros_ws/src/install_spot_ros2.sh

# Build packages with Colcon
WORKDIR /ros_ws/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-ignore proto2ros proto2ros_tests
