#!/usr/bin/env bash

xhost +local:docker

docker run -it --rm \
    --net host \
    --name vcam_test_container \
    -e DISPLAY=$DISPLAY \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -u user  \
 vcam_image:foxy bash -c \
 "ros2 run virtual_camera virtual_camera"

xhost -local:docker
