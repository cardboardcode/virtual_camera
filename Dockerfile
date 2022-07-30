FROM ros:humble-ros-base-jammy

MAINTAINER Bey Hao Yun <beyhy94@gmail.com>

# Add user
RUN adduser --quiet --disabled-password guiuser

# Install OpenCV
RUN mv /etc/apt/trusted.gpg /etc/apt/trusted.gpg.d/ && \
    apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y ros-humble-cv-bridge && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/guiuser/

# Get ROS2 package image_tools available for sourcing.
USER guiuser
RUN git clone https://github.com/ros2/demos.git \
    --branch humble \
    --single-branch \
    --depth 1

WORKDIR /
