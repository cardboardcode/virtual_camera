#!/usr/bin/env bash

sudo apt-get update
sudo apt-get install -y python3-pip
pip install -U colcon-common-extensions

colcon version-check

rosdep update
rosdep install -y --from-paths ./ --ignore-src --rosdistro humble

source /opt/ros/humble/setup.bash
colcon build
