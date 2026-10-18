<a href="https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml"><img src="https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml/badge.svg" alt="build" height="28" /></a>
<img src="https://img.shields.io/badge/Rust-2026-DEA584?style=for-the-badge&logo=rust&logoColor=white" alt="Rust" height="28" />

## **What Is This**?

A pure **Rust** ROS 2 package that **simulates a camera feed** by streaming static video files or static images as standard ROS 2 `sensor_msgs/msg/Image` messages over DDS via `roslib-rust` (`roslibrust`).

## **Dependencies** :books:

1. Docker
2. ROS 2 [Rolling Ridley](https://docs.ros.org/en/rolling/index.html)
3. Rust 1.85+ (when building on host)
4. `roslibrust`, `tokio`, `opencv` (Rust crate)

## **Build** :hammer:

### **1. Build Docker Image**

```bash
bash scripts/1_build_docker_image.bash
```

## **Configure** :wrench:

Use the interactive helper to select which media file in `data/` gets published on the ROS 2 topic. It lists the available files and points `input_data` at your choice:

```bash
bash scripts/set_input_data.bash
```

To update the selection later, simply place your new media file in `data/` and re-run the script.

## **Run** :rocket:

```bash
bash scripts/2_create_docker_container.bash
```
