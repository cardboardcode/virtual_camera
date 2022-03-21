#!/bin/env bash

source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch virtual_camera showimageraw.launch.py
