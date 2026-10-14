<a href="https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml"><img src="https://github.com/cardboardcode/virtual_camera/actions/workflows/ci.yaml/badge.svg" alt="build" height="28" /></a>
<img src="https://img.shields.io/badge/Rust-2026-DEA584?style=for-the-badge&logo=rust&logoColor=white" alt="Rust" height="28" />

> [!WARNING]
> This branch is still unverified and not working in general. Do not use.

## **What Is This**?

A pure **Rust** ROS 2 package that **simulates a camera feed** by streaming static video files or static images as standard ROS 2 `sensor_msgs/msg/Image` messages over DDS via `roslib-rust` (`roslibrust`).

## **Dependencies** :books:

1. Docker
2. ROS 2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html) **or** [Rolling Ridley](https://docs.ros.org/en/rolling/index.html) (choose via `ROS_DISTRO`, default `jazzy`)
3. Rust 1.85+ (when building on host)
4. `roslibrust`, `tokio`, `opencv` (Rust crate)

The package is **distro-parameterized**: the plain `ros:<distro>-perception` base image and a
`build.rs` that reads ROS schemas from `/opt/ros/$ROS_DISTRO` mean one source tree builds for both.

## **Build** :hammer:

### **1. Build Docker Image**

```bash
bash scripts/1_build_docker_image.bash            # Jazzy (default) -> vcam_image:jazzy
ROS_DISTRO=rolling bash scripts/1_build_docker_image.bash    # Rolling -> vcam_image:rolling
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
