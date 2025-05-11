
[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![codecov](https://codecov.io/gh/cardboardcode/virtual_camera/graph/badge.svg?token=DITZXL86DN)](https://codecov.io/gh/cardboardcode/virtual_camera)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

## **What Is This**?

A ROS2 package that **simulates a camera**, providing ROS messages from **playing a static video or image**.

⚠️ This is an experimental Rust-based implementation that still uses memory-unsafe function calls. Use it at your own risk.

## **Dependencies**

1. [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
2. ROS2 [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## **Setup** :hammer:

**Run** the command below.

```bash
WIP
```

## **Run** :rocket:

Run the commands below to run `virtual_camera` ROS 2 node:

```bash
WIP
```

#### **Run Options**

```bash
WIP
```

#### **Create A Static Video/Image**

```bash

```

```bash

```

#### **Control FPS**
Run the following command to control the speed of the video

```bash
WIP
```

#### **Docker Instructions** [Optional] :whale2:

This section is for **users who do not want to worry about installing all dependencies stated above**.

**Build** the docker image.

```bash
docker build --tag vcam_image:humble_rust .
```

**Enable** X11-forwarding for showing GUI application from within docker container on host machine:

```bash
xhost +local:docker
```

**Run** the docker image.

```bash
docker run -it --rm \
    --net host \
    --name vcam_test_container \
    -e DISPLAY=$DISPLAY \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./data:/workspace/data \
    -u user  \
 vcam_image:humble_rust bash -c \
 "ros2 launch virtual_camera run.launch"
```
