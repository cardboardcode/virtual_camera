#!/bin/env bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch virtual_camera showimageraw.launch.py use_image_viewer:=true
