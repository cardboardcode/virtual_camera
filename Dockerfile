FROM ros:humble-ros-base-jammy

MAINTAINER Bey Hao Yun <beyhy94@gmail.com>

# Add user
RUN adduser --quiet --disabled-password user

# Install OpenCV
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y ros-humble-cv-bridge && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/user/

# Get ROS2 package image_tools available for sourcing.
USER user
RUN git clone https://github.com/ros2/demos.git \
    --branch humble \
    --single-branch \
    --depth 1

WORKDIR /home/user
