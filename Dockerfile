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

USER user

WORKDIR /home/user
