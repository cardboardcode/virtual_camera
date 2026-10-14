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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT"

# Target distro (default rolling; override: ROS_DISTRO=jazzy bash this-script)
ROS_DISTRO="${ROS_DISTRO:-rolling}"
IMAGE_NAME="vcam_image:${ROS_DISTRO}"

echo "=================================================="
echo " STEP 1: Building Docker image ($IMAGE_NAME)"
echo "=================================================="
docker build --build-arg ROS_DISTRO="$ROS_DISTRO" -t "$IMAGE_NAME" .

echo "=================================================="
echo " STEP 2: Running Cargo Unit Tests in Container"
echo "=================================================="
docker run --rm \
    -e ROS_DISTRO="$ROS_DISTRO" \
    -v "$REPO_ROOT/data":/virtual_camera_ws/src/virtual_camera/data \
    "$IMAGE_NAME" \
    cargo test --lib --test camera_tests -- --nocapture

echo "=================================================="
echo " STEP 3: Running Headless GUI Tests (Xvfb)"
echo "=================================================="
docker run --rm \
    -e ROS_DISTRO="$ROS_DISTRO" \
    -v "$REPO_ROOT/data":/virtual_camera_ws/src/virtual_camera/data \
    "$IMAGE_NAME" \
    bash -c 'Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp & XVFB_PID=$!; sleep 1; DISPLAY=:99 cargo test --test viewer_tests -- --nocapture; kill $XVFB_PID 2>/dev/null || true'

echo "=================================================="
echo " STEP 4: Running ROS 2 End-to-End Topic Test"
echo "=================================================="
docker run --rm \
    --network host \
    -e ROS_DISTRO="$ROS_DISTRO" \
    -v "$REPO_ROOT/data":/virtual_camera_ws/src/virtual_camera/data \
    -v "$REPO_ROOT/scripts":/virtual_camera_ws/src/virtual_camera/scripts:ro \
    "$IMAGE_NAME" \
    bash /virtual_camera_ws/src/virtual_camera/scripts/test_ros2_e2e.bash

echo "=================================================="
echo " ALL TESTS COMPLETED AND VERIFIED IN CONTAINER!   "
echo "=================================================="
