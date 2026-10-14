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

source /opt/ros/jazzy/setup.bash

echo "=== [1/4] Starting rosbridge_websocket server ==="
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!

cleanup() {
    echo "Cleaning up background processes..."
    if [ -n "$VCAM_PID" ]; then
        kill "$VCAM_PID" 2>/dev/null || true
    fi
    if [ -n "$ROSBRIDGE_PID" ]; then
        kill "$ROSBRIDGE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "Waiting for rosbridge server on port 9090..."
attempts=0
while [ "$attempts" -lt 30 ]; do
    if (echo > /dev/tcp/127.0.0.1/9090) >/dev/null 2>&1; then
        echo "rosbridge server is listening on port 9090."
        break
    fi
    attempts=$(( attempts + 1 ))
    sleep 0.5
done

if ! (echo > /dev/tcp/127.0.0.1/9090) >/dev/null 2>&1; then
    echo "ERROR: rosbridge server failed to start within timeout."
    cat /tmp/rosbridge.log
    exit 1
fi

echo "=== [2/4] Starting virtual_camera (Rust) ==="
virtual_camera --fps 24 --ws-url ws://localhost:9090 > /tmp/vcam.log 2>&1 &
VCAM_PID=$!
sleep 2

echo "=== [3/4] Checking ROS 2 topic existence ==="
TOPIC="/virtual_camera/image_raw"
if ros2 topic list | grep -q "$TOPIC"; then
    echo "Found topic $TOPIC in ROS 2 graph."
else
    echo "ERROR: Topic $TOPIC not found in ros2 topic list."
    cat /tmp/vcam.log
    exit 1
fi

TYPE=$(ros2 topic type "$TOPIC")
echo "Topic type: $TYPE"
if [ "$TYPE" != "sensor_msgs/msg/Image" ]; then
    echo "ERROR: Unexpected topic type: $TYPE"
    exit 1
fi

echo "=== [4/4] Verifying message reception via ros2 topic echo ==="
TIMEOUT_SEC=10
if timeout "$TIMEOUT_SEC" ros2 topic echo "$TOPIC" --once > /tmp/echo.log 2>&1; then
    echo "Successfully received Image message from $TOPIC!"
    head -n 15 /tmp/echo.log
else
    echo "ERROR: Timed out waiting for message on $TOPIC."
    cat /tmp/vcam.log
    exit 1
fi

echo "=== ALL ROS 2 E2E CHECKS PASSED SUCCESSFULLY ==="
