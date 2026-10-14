[![build](https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml/badge.svg)](https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml)
![Rust](https://img.shields.io/badge/Rust-2026-DEA584?style=for-the-badge&logo=rust&logoColor=white)

## **What Is This**?

A pure **Rust** ROS 2 package that **simulates a camera feed** by streaming static video files or static images as standard ROS 2 `sensor_msgs/msg/Image` messages over DDS via `roslib-rust` (`roslibrust`).

## **Dependencies** :books:

1. Docker
2. ROS 2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
3. Rust 1.85+ (when building on host)
4. `roslibrust`, `tokio`, `opencv` (Rust crate)

## **Build** :hammer:

### **1. Build Docker Image**
```bash
bash scripts/1_build_docker_image.bash
```

## **Configure**

Set your custom video media file to be published on ROS 2 topic:

```bash
cd data

# Point to sample video
ln -sf video.mp4 input_data

# Point to sample image
ln -sf nadezhda_diskant.jpg input_data
```

## **Run** :rocket:

```bash
bash scripts/2_create_docker_container.bash
```
