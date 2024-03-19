#!/usr/bin/env bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y lcov curl
sudo apt-get install -y python3-pip
pip install -U colcon-common-extensions
pip install -U colcon-lcov-result

# Install ROS 2 dependencies
rosdep update
rosdep install -y --from-paths ./ --ignore-src --rosdistro rolling

# Build virtual_camera ROS 2 package
source /opt/ros/rolling/setup.bash
colcon build --symlink-install --cmake-args \
            -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' \
            -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'

# Generate code coverage report of virtual_camera ROS 2 package gtests
colcon lcov-result --initial
colcon test --packages-select virtual_camera
colcon lcov-result
