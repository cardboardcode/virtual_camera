#!/bin/env bash

source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch virtual_camera showimageraw.launch.py
