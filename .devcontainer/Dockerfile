FROM ros:humble-ros-base-jammy

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO
ARG INSTALL_PACKAGE=base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    lcov \
    curl \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    libpython3-dev \
    python3-vcstool \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tl-expected && \
    rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

ARG SPOT_SDK_VERSION="4.1.0"
ARG SPOT_MSG_VERSION="${SPOT_SDK_VERSION}-4"

# Install bosdyn_msgs package
RUN curl -sL https://github.com/bdaiinstitute/bosdyn_msgs/releases/download/${SPOT_MSG_VERSION}/ros-humble-bosdyn_msgs_${SPOT_MSG_VERSION}-jammy_amd64.run --output /tmp/ros-humble-bosdyn_msgs_${SPOT_MSG_VERSION}-jammy_amd64.run --silent \
  && chmod +x /tmp/ros-humble-bosdyn_msgs_${SPOT_MSG_VERSION}-jammy_amd64.run \
  && ((yes || true) | /tmp/ros-humble-bosdyn_msgs_${SPOT_MSG_VERSION}-jammy_amd64.run) \
  && rm /tmp/ros-humble-bosdyn_msgs_${SPOT_MSG_VERSION}-jammy_amd64.run

# Install spot_cpp_sdk package
RUN curl -sL https://github.com/bdaiinstitute/spot-cpp-sdk/releases/download/v${SPOT_SDK_VERSION}/spot-cpp-sdk_${SPOT_SDK_VERSION}_amd64.deb --output /tmp/spot-cpp-sdk_${SPOT_SDK_VERSION}_amd64.deb --silent \
  && dpkg -i /tmp/spot-cpp-sdk_${SPOT_SDK_VERSION}_amd64.deb \
  && rm /tmp/spot-cpp-sdk_${SPOT_SDK_VERSION}_amd64.deb

# Install bosdyn_msgs missing dependencies
RUN python -m pip install --no-cache-dir --upgrade pip==22.3.1 \
    && pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 \
    numpy==1.24.1 \
    pytest-cov==4.1.0 \
    pytest-xdist==3.5.0 \
    bosdyn-api==${SPOT_SDK_VERSION} \
    bosdyn-core==${SPOT_SDK_VERSION} \
    bosdyn-client==${SPOT_SDK_VERSION} \
    bosdyn-mission==${SPOT_SDK_VERSION} \
    bosdyn-choreography-client==${SPOT_SDK_VERSION} \
    pip cache purge

# Install spot_wrapper requirements
RUN --mount=type=bind,source=spot_wrapper,target=/tmp/context/spot_wrapper cd /tmp/context && \
    pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 -r spot_wrapper/requirements.txt && \
    pip cache purge

# Install packages dependencies
RUN --mount=type=bind,source=.,target=/tmp/context \
    apt-get update -q && rosdep update && \
    rosdep install -y -i --from-paths /tmp/context --skip-keys "bosdyn bosdyn_msgs spot_wrapper" && \
    rm -rf /var/lib/apt/lists/*

# ROS doesn't recognize the docker shells as terminals so force colored output
ENV RCUTILS_COLORIZED_OUTPUT=1
