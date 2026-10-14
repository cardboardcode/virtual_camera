#!/usr/bin/env bash
# Copyright 2026 Bey Hao Yun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

# Target distro (default rolling; override: ROS_DISTRO=jazzy bash this-script)
ROS_DISTRO="${ROS_DISTRO:-rolling}"
IMAGE_TAG="vcam_image:${ROS_DISTRO}"

xhost +local:root || true
xhost +local:docker || true

docker run -it --rm \
    --network host \
    --name vcam_test_container \
    -e DISPLAY="$DISPLAY" \
    -e ROS_DISTRO="$ROS_DISTRO" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/data":/virtual_camera_ws/src/virtual_camera/data \
    "$IMAGE_TAG" bash -c \
    "ros2 launch virtual_camera showimageraw.launch.py use_image_viewer:=true"

xhost -local:docker || true
xhost -local:root || true
