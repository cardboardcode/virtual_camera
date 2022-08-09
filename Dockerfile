FROM ros:foxy-ros-base-focal

MAINTAINER Bey Hao Yun <beyhy@artc.a-star.edu.sg>

# Add user
RUN adduser --quiet --disabled-password guiuser

# Install OpenCV
RUN apt-get update && \
    apt-get install -y libopencv-dev && \
    apt-get install -y ros-foxy-cv-bridge && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/guiuser/

# Get ROS2 package image_tools available for sourcing.
USER guiuser
RUN git clone https://github.com/ros2/demos.git \
    --branch foxy \
    --single-branch \
    --depth 1

WORKDIR /
