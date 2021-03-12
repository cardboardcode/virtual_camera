
[![CI](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## **What Is This**?

A ROS2 package that **simulates a camera**, providing ROS messages from **playing a static video**.

## **Dependencies**

1. OpenCV
2. ROS2 Foxy

## **Setup**

Run the command below.

```bash
ros2 run virtual_camera virtual_camera
```

```bash
# For running without image-viewer
./scripts/run.sh
# For running with image-viewer
./scripts/show_image.bash
```

### **Control FPS**
Run the following command to control the speed of the video

```bash
ros2 param set /virtual_camera FPS <an integer>
# Eg. ros2 pararm set /virtual_camera FPS 24

```

### **Docker Instructions**

This section is for users who do not want to worry about installing all dependencies stated above.

1. Build the docker image.

```bash
docker build --tag vcam_image .
```

2. Run the docker image.

```bash
# For first run.
docker run -it \
--name vcam_test_container \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v $(pwd):/home/guiuser/virtual_camera \
-u 0  \
vcam_image:latest /bin/bash

# For subsequent run.
docker start vcam_test_container && docker exec -it vcam_test_container bash

```
