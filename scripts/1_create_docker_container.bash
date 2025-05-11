#!/usr/bin/env bash

xhost +local:docker

docker run -it --rm \
    --net host \
    --name vcam_test_container \
    -e DISPLAY=$DISPLAY \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./data:/workspace/data \
    -u user  \
 vcam_image:humble_rust bash -c \
 "ros2 launch virtual_camera run.launch.py \
 use_image_viewer:=True"

xhost -local:docker
