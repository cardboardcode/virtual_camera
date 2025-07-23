#!/usr/bin/env bash

xhost +local:docker

docker run -it --rm \
    --network host \
    --name vcam_test_container \
    -e DISPLAY=$DISPLAY \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./data:/virtual_camera_ws/src/virtual_camera/data \
vcam_image:jazzy bash -c \
"ros2 launch virtual_camera showimageraw.launch.py \
use_image_viewer:=true"

xhost -local:docker