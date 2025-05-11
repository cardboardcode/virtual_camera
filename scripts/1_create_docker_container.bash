#!/usr/bin/env bash

read -p "Run [virtual_camera] with image viewer: [y/n]: " response

case "$response" in
  [yY])
    value="True"
    echo "You entered 'y'. Variable 'value' set to: $value"
    ;;
  [nN])
    value="False"
    echo "You entered 'n'. Variable 'value' set to: $value"
    ;;
  *)
    echo "Invalid input. Please enter 'y' or 'n'."
    exit 1 # Exit with an error code
    ;;
esac

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
 use_image_viewer:=${value}"

xhost -local:docker

unset value response
