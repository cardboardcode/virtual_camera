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

# Adjust the publish FPS for a virtual_camera session.
#
# Historical note: this script previously called `ros2 param set /virtual_camera
# FPS <n>`, which cannot work - FPS is a CLI argument to the `virtual_camera`
# binary (see src/cli.rs), not a ROS 2 parameter. The correct way to change the
# frame rate is to relaunch the node with the new `--fps` value.
#
# This helper prompts for the FPS, validates it, and prints the exact command to
# use. The caller runs it in the session where `virtual_camera` currently runs
# (e.g. inside the Docker container).
set -eo pipefail

source "/opt/ros/${ROS_DISTRO:-rolling}/setup.bash"

read -rp "Please input the FPS integer (1-200): " input
if ! [[ "$input" =~ ^[0-9]+$ ]] || (( input < 1 || input > 200 )); then
    echo "ERROR: FPS must be an integer in [1, 200] (got: ${input})" >&2
    exit 1
fi

WS_URL="${VCAM_WS_URL:-ws://localhost:9090}"
echo
echo "Relaunch virtual_camera with the new frame rate:"
echo "  ros2 launch virtual_camera showimageraw.launch.py \\"
echo "      fps:=${input} \\"
echo "      ws_url:=${WS_URL}"
echo
echo "Or directly:"
echo "  virtual_camera --fps ${input} --ws-url ${WS_URL}"
