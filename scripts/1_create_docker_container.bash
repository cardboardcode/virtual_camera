#!/usr/bin/env bash

xhost +local:docker

docker run -ti \
--ipc host \
--net host \
--name vcam_test_container \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev/shm:/dev/shm \
-v $(pwd):/home/user/virtual_camera \
-u 0  \
 vcam_image:latest /bin/bash

xhost -local:docker
