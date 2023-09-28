
[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![codecov](https://codecov.io/gh/cardboardcode/virtual_camera/branch/main/graph/badge.svg?token=DITZXL86DN)](https://codecov.io/gh/cardboardcode/virtual_camera)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

## **What Is This**?

A ROS2 package that **simulates a camera**, providing ROS messages from **playing a static video or image**.

## **Dependencies**

1. [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
2. ROS2 [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## **Setup**

**Run** the command below.

```bash
cd $HOME
git clone https://github.com/cardboardcode/virtual_camera.git --branch humble_devel --single-branch --depth 1
cd ~/virtual_camera
source /opt/ros/humble/setup.bash
colcon build
```

```bash
cd ~/virtual_camera
source install/setup.bash
ros2 run virtual_camera virtual_camera

ros2 launch virtual_camera showimageraw.launch.py
```

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

#### **Control FPS**
Run the following command to control the speed of the video

```bash
ros2 param set /virtual_camera FPS <an integer>
# Eg. ros2 pararm set /virtual_camera FPS 24

```

#### **Docker Instructions**

This section is for **users who do not want to worry about installing all dependencies stated above**.

**Build** the docker image.

```bash
docker build --tag vcam_image .
```

**Run** the docker image.

```bash
# For first run.
sudo docker run -ti \
--name vcam_test_container \
-v $(pwd):/home/user/virtual_camera \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-u 0  \
 vcam_image:latest /bin/bash

# For subsequent run.
docker start vcam_test_container && docker exec -it vcam_test_container bash

```
