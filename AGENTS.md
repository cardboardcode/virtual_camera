# AGENTS.md

> **Repository:** `virtual_camera`  
> **Status:** Active development (Rust implementation, `rolling` branch)  
> **Target Platform:** ROS 2 **Rolling Ridley** on Ubuntu 26.04 LTS (Resolute Raccoon)  
> **Maintainer:** Bey Hao Yun (`cardboardcode`, `beyhy@artc.a-star.edu.sg`)  

---

## 1. System Overview & Architecture

`virtual_camera` is a pure **Rust** ROS 2 package designed to simulate a camera feed by reading static media (video files or static images) via OpenCV and streaming them as standard ROS 2 `sensor_msgs/msg/Image` messages over DDS via `roslib-rust` (`roslibrust`).

### Core Architectural Concepts

1. **Dual-Mode Media Streaming**:
   - The primary node (`virtual_camera`) attempts to stream a video file from `data/input_data`.
   - If the file is not a valid video or cannot be opened by OpenCV's `VideoCapture`, it automatically falls back to static image mode (`imread`).
   - If `data/input_data` is missing or unreadable, it defaults to `data/nadezhda_diskant.jpg`.
   - Video playback loops indefinitely: when reaching end-of-stream (empty frame), it re-opens `data/input_data` and continues publishing.

2. **Asynchronous ROS 2 Communication with `roslib-rust`**:
   - The node utilizes `roslibrust` on top of the Tokio async runtime.
   - It communicates with the ROS 2 graph via `rosbridge_server` (`ws://localhost:9090`), publishing `sensor_msgs/msg/Image` directly onto the CycloneDDS bus.

3. **Intra-Process & Standalone Viewer**:
   - In-process: when `--use-image-viewer` is passed, `ImageViewer` runs as an asynchronous Tokio task alongside the publisher.
   - Standalone: a dedicated `image_viewer` binary is available to run as an independent subscriber process.

4. **Dynamic Frame Rate**:
   - The CLI argument `--fps <int>` (default `24`) configures the publication interval (`1000 / fps` ms).

---

## 2. Repository Layout

```text
virtual_camera/
├── Cargo.toml                          # Rust package manifest & dependencies
├── Cargo.lock                          # Pinned dependency versions
├── package.xml                         # ROS package manifest (format 3)
├── CONTRIBUTING.md                     # DCO sign-off & Apache 2.0 contribution terms
├── LICENSE                             # Apache License 2.0
├── README.md                           # User documentation & setup instructions
├── AGENTS.md                           # Operational guide for AI agents
├── IMPLEMENTATION_PLAN.md              # Phased Rust migration plan
├── Dockerfile                          # Container build (ros:rolling-perception)
├── .dockerignore                       # Excludes build artifacts from Docker context
├── data/
│   ├── nadezhda_diskant.jpg            # Default fallback test image
│   └── video.mp4                       # Sample test video
├── launch/
│   └── showimageraw.launch.py          # Launch file with arguments (use_image_viewer, use_debug, fps)
├── src/
│   ├── lib.rs                          # Core crate library: MediaSource, frame conversion, cursor spinner
│   ├── camera.rs                       # VirtualCamera publisher node logic
│   ├── viewer.rs                       # ImageViewer subscriber node logic
│   └── bin/
│       ├── virtual_camera.rs           # Entrypoint for virtual_camera executable
│       └── image_viewer.rs             # Entrypoint for standalone image_viewer executable
├── tests/
│   ├── camera_tests.rs                 # Unit & integration tests for media fallback and conversions
│   └── viewer_tests.rs                 # Tests for viewer and headless windowing
└── scripts/
    ├── 1_build_docker_image.bash       # Builds Docker image vcam_image:rolling
    ├── 2_create_docker_container.bash  # Runs Docker container with X11 GUI forwarding
    ├── run_container_tests.bash        # Automated test suite running inside Docker
    ├── test_ros2_e2e.bash              # In-container end-to-end DDS topic verification
    ├── build.bash                      # Runs cargo build --release
    ├── run.bash                        # Runs virtual_camera node
    ├── show_image.bash                 # Runs virtual_camera with viewer enabled
    ├── change_fps.bash                 # CLI helper to adjust FPS
    └── set_input_data.bash             # Interactive CLI script to symlink media to data/input_data
```

---

## 3. Technology Stack & Dependencies

- **Language & Standards:** Rust 2021 edition (`rustc` 1.85+), Python 3 (launch files & tooling).
- **ROS 2 Middleware:** ROS 2 Rolling Ridley (`rosbridge_server`, `rmw_cyclonedds_cpp`, `sensor_msgs`, `std_msgs`).
- **Rust Client:** `roslibrust` (0.26+) with `tokio` (1.38+).
- **Computer Vision:** OpenCV (`opencv` Rust bindings v0.101 linking to system `libopencv-dev`).
- **Testing:** `cargo test`, `xvfb-run` for headless GUI verification, in-container ROS 2 topic verification.

---

## 4. Build, Run, and Test Workflows

### Building the Docker Container
```bash
bash scripts/1_build_docker_image.bash
```

### Running Automated Tests in Container
```bash
bash scripts/run_container_tests.bash
```

### Running the Container with GUI Forwarding
```bash
bash scripts/2_create_docker_container.bash
```

### Local Compilation (Host with ROS 2 & Rust)
```bash
source /opt/ros/rolling/setup.bash
cargo build --release
```

---

## 5. Node Interfaces & Topics

### Published Topics

| Topic | Type | Frame ID | Description |
| :--- | :--- | :--- | :--- |
| `/virtual_camera/image_raw` | `sensor_msgs/msg/Image` | `"world"` | Streamed camera frames (BGR8 encoding, published at `1000 / FPS` ms intervals). |

### Supported Encodings in `ImageViewer`
- `mono8`
- `bgr8`
- `rgb8` (converted to BGR for display)
- `mono16`
- `rgba8`
- `bgra8`
- `32FC1`

---

## 6. Critical Constraints & Invariants for Agents

1. **Docker Container Execution**:
   - Automated tests and verification must be verified using `scripts/run_container_tests.bash` inside the Docker container.
2. **Headless Safety**:
   - Any GUI-related OpenCV test must use `xvfb-run` or headless verification to avoid crashing in display-less environments.
3. **Commit Standards**:
   - DCO sign-off (`git commit -s`).
   - Gitmoji commit prefixes (`:sparkles:`, `:hammer:`, `:recycle:`, `:memo:`, `:bug:`).
   - Apache 2.0 license headers on all newly created source files.
