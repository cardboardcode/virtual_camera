FROM ros:jazzy-perception-noble

MAINTAINER Bey Hao Yun <beyhy94@gmail.com>

# Install OpenCV
RUN apt-get update && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/user
