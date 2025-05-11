
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

⚠️ Note that ROS2 Rust client library is still in early development with no official binaries released on buildfarm.

Therefore, you will need to build certain ROS 2 packages from scratch

1. Create a ROS 2 workspace

```bash
cd $HOME && mkdir -p ros2_rust_ws/src
```

```bash
cd ros2_rust_ws/src 
```

```bash
git clone https://github.com/ros2-rust/ros2_rust.git --depth 1 --branch main --single-branch && cd $HOME/ros2_rust_ws
```

```bash
vcs import src < src/ros2_rust/ros2_rust_humble.repos
```

```bash
source /opt/ros/humble/setup.bash && colcon build
```

```bash
git clone https://github.com/cardboardcode/virtual_camera.git --branch humble_rust_devel --depth 1 --single-branch src/virtual_camera
```

```bash
source install/setup.bash && colcon build --packages-select virtual_camera
```


**Reference**: https://github.com/ros2-rust/ros2_rust/blob/main/docs/building.md

## **Run** :rocket:

Run the commands below to run `virtual_camera` ROS 2 node:

```bash
source install/setup.bash
```

```bash
ros2 launch virtual_camera run.launch.py use_image_viewer:=True
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
