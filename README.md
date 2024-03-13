[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## **What Is This**?

A ROS2 package that **simulates a camera**, providing ROS messages from **playing a static video or image**.

## **Dependencies**

1. [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
2. ROS2 [Rolling Ridley](https://docs.ros.org/en/rolling/Installation.html)
3. [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2)

## **Tested on**

- Windows **10**
- [Windows Subsystem for Linux](https://ubuntu.com/desktop/wsl)

## **Setup**
This section provides instructions on how to build **vision_opencv** since the default ROS2 Foxy binary distribution does not automatically provide it.

1. **Build** vision_opencv ROS2 package.

```cmd
mkdir -p c:\vcam_ws\src
cd c:\vcam_ws\src
git clone https://github.com/flynneva/vision_opencv.git --single-branch --branch ros2 --depth 1
git clone https://github.com/cardboardcode/virtual_camera.git --single-branch --branch windows_develop --depth 1
cd c:\vcam_ws\
call c:\opt\ros\rolling\local_setup.bat
colcon build --merge-install
call install\setup.bat
```

2. **Run** the command below.

```bash
ros2 launch virtual_camera showimageraw.launch.py
```

```bash
# For running without image-viewer
./scripts/run.sh
# For running with image-viewer
./scripts/show_image.bash
```

#### **Create A Static Video/Image**

```bash
# Jump into a folder called data
cd data
# Move your intended video/image into this folder
# Create a symbolic link to video.
mklink <video_file_name> input_data
# Or create a symbolic link to image.
mklink <image_file_name> input_data
```
If you encounter the error `You do not have sufficient privilege to perform this operation.`, run the above commands in **Administrator: Command Prompt**.

#### **Control FPS**
Run the following command to control the speed of the video

```bash
ros2 param set /virtual_camera FPS <an integer>
# Eg. ros2 pararm set /virtual_camera FPS 24

```
