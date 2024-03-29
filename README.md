
[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![codecov](https://codecov.io/gh/cardboardcode/virtual_camera/graph/badge.svg?token=DITZXL86DN)](https://codecov.io/gh/cardboardcode/virtual_camera)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

## **What Is This**?

A ROS2 package that **simulates a camera**, providing ROS messages from **playing a static video or image**.

## **Dependencies**

1. [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
2. ROS2 [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## **Setup** :hammer:

**Run** the command below.

```bash
cd $HOME
git clone https://github.com/cardboardcode/virtual_camera.git --branch humble_devel --single-branch --depth 1
cd ~/virtual_camera
source /opt/ros/humble/setup.bash
colcon build
```

## **Run** :rocket:

Run the commands below to run `virtual_camera` ROS 2 node:

```bash
cd ~/virtual_camera
source install/local_setup.bash
```

#### **Run Options**

```bash
# For running without image-viewer
./scripts/run.bash
# For running with image-viewer
./scripts/show_image.bash
```

#### **Create A Static Video/Image**

```bash
# Jump into a folder called data
cd data
# Move your intended video/image into this folder
# Create a symbolic link to video.
ln -sf <video_file_name> input_data
# Or create a symbolic link to image.
ln -sf <image_file_name> input_data
```

```bash
# You can choose not to do it manually by running the following script:
# Input the number of the input image shown in the script -
./scripts/set_input_data.bash
```

#### **Control FPS**
Run the following command to control the speed of the video

```bash
ros2 param set /virtual_camera FPS <an integer>
# Eg. ros2 pararm set /virtual_camera FPS 24

```

#### **Docker Instructions** [Optional] :whale2:

This section is for **users who do not want to worry about installing all dependencies stated above**.

**Build** the docker image.

```bash
docker build --tag vcam_image:humble .
```

**Run** the docker image.

```bash
# Enable display to be forwarded from container to host.
xhost +local:docker
# For first run.
docker run -it \
--ipc host \
--net host \
--name vcam_humble_test_container \
-v $(pwd):/home/user/virtual_camera \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-u 1000  \
 vcam_image:humble /bin/bash

# For subsequent run.
docker start vcam_humble_test_container && docker exec -it vcam_humble_test_container bash

```
