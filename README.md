[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/industrial_ci_action.yml)
[![codecov](https://codecov.io/gh/cardboardcode/virtual_camera/graph/badge.svg?token=DITZXL86DN)](https://codecov.io/gh/cardboardcode/virtual_camera)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Rust](https://img.shields.io/badge/Rust-2021-DEA584?style=for-the-badge&logo=rust&logoColor=white)

## **What Is This**?

A pure **Rust** ROS 2 package that **simulates a camera feed** by streaming static video files or static images as standard ROS 2 `sensor_msgs/msg/Image` messages over DDS via `roslib-rust` (`roslibrust`).

## **Key Features**

- **Asynchronous Architecture:** Built on Rust's `tokio` async runtime with `roslibrust`.
- **Dual-Mode Media Streaming:** Automatically detects and loops video streams (`data/input_data`), with fallback to static images (`data/nadezhda_diskant.jpg`).
- **Dynamic FPS Configuration:** Adjustable frame rate via `--fps` CLI argument or dynamic configuration.
- **Embedded & Standalone Viewer:** Support for in-process or standalone `image_viewer` displaying standard encodings (`bgr8`, `rgb8`, `mono8`, `rgba8`, `mono16`, `32FC1`).
- **Container-First:** Designed to build, run, and test seamlessly inside Docker (`ros:jazzy-perception-noble`).

## **Dependencies**

1. Docker / Podman
2. ROS 2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
3. Rust 1.85+ (when building on host)
4. `roslibrust`, `tokio`, `opencv` (Rust crate)

## **Quick Start with Docker** :whale:

### **1. Build Docker Image**
```bash
./scripts/1_build_docker_image.bash
```

### **2. Run with Live Image Viewer**
```bash
./scripts/2_create_docker_container.bash
```

### **3. Run Automated Tests Inside Docker**
```bash
./scripts/run_container_tests.bash
```

## **Manual Execution** :rocket:

```bash
# Sourcing ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build Rust workspace
cargo build --release

# Run virtual_camera publisher node
./target/release/virtual_camera --fps 24

# Run with embedded viewer
./target/release/virtual_camera --fps 24 --use-image-viewer

# Run standalone viewer in a separate process
./target/release/image_viewer
```

#### **Configuring Input Media**

```bash
cd data

# Point to sample video
ln -sf video.mp4 input_data

# Point to sample image
ln -sf nadezhda_diskant.jpg input_data
```

## **License**
Licensed under the [Apache License 2.0](LICENSE).
