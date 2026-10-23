# Codespaces / local dev image. The core library builds without ROS; this image
# adds Humble so the wrapper, RViz, and the core tests can be built in one place.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      ninja-build \
      pkg-config \
      libeigen3-dev \
      libyaml-cpp-dev \
      python3-pip \
      python3-numpy \
      python3-scipy \
      python3-matplotlib \
      python3-yaml \
      python3-colcon-common-extensions \
      ros-humble-rclcpp \
      ros-humble-std-msgs \
      ros-humble-geometry-msgs \
      ros-humble-visualization-msgs \
      ros-humble-tf2 \
      ros-humble-tf2-ros \
      ros-humble-ament-cmake \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-launch-ros \
      ros-humble-rviz2 \
      ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir control

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
CMD ["bash"]
