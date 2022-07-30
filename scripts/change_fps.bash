#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

echo "Please input the FPS integer (1-200)"
read input

ros2 param set /virtual_camera FPS $input
