FROM ros:rolling-ros-base-focal

MAINTAINER Bey Hao Yun <beyhy94@gmail.com>

# Add user
RUN adduser --quiet --disabled-password user

# Install OpenCV
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y ros-foxy-cv-bridge && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/user/

# Get ROS2 package image_tools available for sourcing.
USER user
RUN git clone https://github.com/ros2/demos.git \
    --branch rolling_devel \
    --single-branch \
    --depth 1

WORKDIR /home/user
