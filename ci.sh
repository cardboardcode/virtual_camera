#!/usr/bin/env bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y lcov curl
sudo apt-get install -y python3-pip
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y python3-colcon-lcov-result

# Install ROS 2 dependencies
rosdep update
rosdep install -y --from-paths ./ --ignore-src --rosdistro jazzy

# Build virtual_camera ROS 2 package
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
            -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' \
            -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'

# Generate code coverage report of virtual_camera ROS 2 package gtests
colcon lcov-result --initial 
colcon test --packages-select virtual_camera --rerun-failed --output-on-failure
colcon lcov-result
