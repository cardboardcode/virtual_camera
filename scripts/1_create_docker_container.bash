#!/usr/bin/env bash

xhost +local:docker

docker run -it --rm \
    --net host \
    --name vcam_test_container \
    -e DISPLAY=$DISPLAY \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -u user  \
 vcam_image:jazzy bash -c \
 "ros2 launch virtual_camera showimageraw.launch.py \
 use_image_viewer:=true"

xhost -local:docker
