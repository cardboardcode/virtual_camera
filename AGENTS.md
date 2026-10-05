# AGENTS.md

Guidance for AI agents (and humans) working in this repository. Describes the codebase **as-built** on the `humble_rust_devel` branch.

## Project Overview

`virtual_camera` is a ROS 2 (Humble) package written in **Rust** that simulates a camera: it plays a static video or image file and publishes it as `sensor_msgs/Image` on `/virtual_camera/image_raw`, with an optional `cv::imshow` viewer that subscribes to the same topic.

- **Build system:** `ament_cargo` (colcon plugin), so it builds inside a colcon workspace. `package.xml` declares `ament_cargo` as build type; `Cargo.toml` is the real package manifest.
- **Dependencies:** `rclrs` (ROS 2 Rust client), `std_msgs`, `sensor_msgs`, `builtin_interfaces`, `anyhow`, `opencv` 0.92 (with `clang-runtime`). `package.xml` pins this to **ROS 2 Humble**.
- **Branch:** `humble_rust_devel` is the primary development branch (README documents cloning it).
- **License:** Apache 2.0 (see `LICENSE`).
- **Safety caveat (from README):** experimental; uses memory-unsafe code paths (see "Known Limitations" below).

## Repository Layout

```
Cargo.toml        Rust package manifest; two [[bin]] targets
src/
  image_publisher.rs   Publisher node: reads data/input_data (video or image), publishes Image
  image_subscriber.rs  Subscriber node: receives Image, displays via OpenCV highgui
package.xml       ament package metadata (ROS name: virtual_camera, version 0.0.0)
launch/
  run.launch.py   Launches image_publisher; conditionally image_subscriber
scripts/          Bash helpers (build, run, docker, coverage)
Dockerfile        Full in-container build of ros2_rust + this package (user "user", workdir /workspace)
data/             Input media + the `input_data` symlink (git-ignored; publisher reads data/input_data)
build/ install/ log/ target/   colcon/cargo artifacts (git-ignored)
CONTRIBUTING.md   Apache-2 license terms + DCO sign-off requirement
codecov.yml       lcov coverage config
```

## Key As-Built Facts (read before editing)

### Binaries & topics

| Binary | Node name | Role |
|---|---|---|
| `image_publisher` | `image_publisher` | Publisher on `/virtual_camera/image_raw` (hardcoded) |
| `image_subscriber` | `image_subscriber` | Subscriber on `/virtual_camera/image_raw` (hardcoded), shows `Image Subscriber` HWindow |

### Data source conventions (surprising — be careful)

- **Hardcoded input path:** `image_publisher.rs::main` reads **`/workspace/data/input_data`** (Docker layout) — a `Path::new(file_name).exists()` check bails out gracefully if missing. It does *not* use a package-relative path or a node parameter.
- `data/input_data` is a **symlink** to a chosen file in `data/` (created via `scripts/set_input_data.bash` or manual `ln -sf <file> input_data`). `.gitignore` ignores all of `data/`.
- File type is sniffed by **magic bytes** (`detect_file_type`): JPEG/PNG/GIF → image, mp4/MPEG/Ogg/WebM/MKV magic → video. Unknown type → neither flag set → the loop publishes nothing.
- "Video" mode actually *loads the file as an image on startup* (`imgcodecs::imread(...).unwrap()`) even when it's a video — the clone (`frame = test_image.clone()`) only happens in image mode, but the read still executes; don't rely on it failing for videos without checking the actual cv::imread behavior.
- Publish loop runs in a spawned thread with `thread::sleep(42 ms)` (≈23 FPS) — **FPS is not parameterized at runtime in the publisher** despite `scripts/change_fps.bash` setting a `/virtual_camera FPS` parameter; that parameter is consumed by no built node. Do not document it as working.
- Frame `header.frame_id` is hardcoded to `"map"`.
- **Encoding support:**
  - Mat → Image: `mono8`, `bgr8` (assumed), `bgra8`, `mono16`; other (depth, channels) combos hit `todo!()` (panics).
  - Image → Mat: `mono8`, `mono16`, `bgr8`, `rgb8` (converted RGB→BGR), `bgra8`, `rgba8` (converted RGBA→BGRA); anything else returns an error.

### Launch

`launch/run.launch.py` exposes one argument:
- `use_image_viewer` (default `True`): when `True`, also launches `image_subscriber`.
- Both nodes run with `emulate_tty=True`, output to screen.

### Docker

- `Dockerfile`: `ros:humble` base, installs OpenCV dev libs, builds **all of ros2_rust** (`vcs import` of `ros2_rust_humble.repos`) before building this package in a second colcon build. Rust toolchain pinned to **1.82.0**. Runs as non-root `user`; appends `source /workspace/install/setup.bash` to `/ros_entrypoint.sh`.
- `scripts/1_create_docker_container.bash` runs the container with `--net host`, X11 forwarding for the viewer, and mounts `./data` → `/workspace/data` (matching the hardcoded publisher path).

### CI / coverage

- `README.md` shows CI build (industrial_ci), codecov, and license badges; the workflow files live on the remote (`.github/` is not present in this checkout).
- Coverage flow: `scripts/generate_cov_report.bash` → colcon build with gcov flags → `colcon test` → `colcon lcov-result`; view with `scripts/view_cov_report.bash` (opens `lcov/index.html` in firefox). `lcov/` is git-ignored.
- `scripts/*.bash`: `build.bash` (colcon build), `run.bash` (launch), `show_image.bash` (launch with viewer true), `set_input_data.bash` (interactive symlink picker), `change_fps.bash` (sets unused param, see above), `0_build_docker_image.bash` / `1_create_docker_container.bash`.

## How to Build & Run

Prereqs: ROS 2 Humble, OpenCV (`libopencv-dev`, `libclang-dev`), colcon plugins `colcon-cargo` + `colcon-ros-cargo`, and ros2_rust built in the same workspace (no official binaries exist; see README for the full vcs-import recipe). The `Dockerfile`/`scripts` above automate this.

```bash
# Build
source /opt/ros/humble/setup.bash
colcon build            # or: scripts/build.bash

# Run (publisher + viewer)
source install/setup.bash
ros2 launch virtual_camera run.launch.py use_image_viewer:=True
# or: scripts/show_image.bash   (viewer)  /  scripts/run.bash   (publisher)

# Point the publisher at new media
ln -sf <media-file> data/input_data   # or scripts/set_input_data.bash

# Coverage
scripts/generate_cov_report.bash [update|clean]
scripts/view_cov_report.bash
```

## Conventions & Process

- **DCO sign-off required:** every commit needs `Signed-off-by: ...` (`scripts` / CONTRIBUTING.md mandate `git commit -s`).
- **License:** Apache 2.0; contributions are licensed as-is (CONTRIBUTING.md).
- **Rust style notes:** `anyhow` is used for error types in the subscriber; the publisher mixes `RclrsError`/`Result` and uses several `.unwrap()`s, `todo!()`, and one `unsafe` block (raw-pointer `Mat` construction from ROS message data). Prefer preserving these patterns over sweeping refactors; there is a `TODO(cardboardcode)` marker where proper `opencv::Error` handling is desired.
- **Do not commit artifacts:** `build/`, `install/`, `log/`, `target/`, `lcov/`, `data/`, and image files are git-ignored.
- **Branching:** work against `humble_rust_devel`; this is the supported ROS 2 Humble line.

## Known Limitations (as-built)

1. Publisher input path is hardcoded to `/workspace/data/input_data` (Docker path) — fails silently (early return) outside Docker unless that path exists.
2. `change_fps.bash` sets a `FPS` parameter that no executable actually reads; FPS is fixed by the 42 ms sleep.
3. `mat_to_ros_image_dynamic_encoding` has `todo!()` for unsupported depth/channel combos (e.g., grayscale 16-bit with alpha, floating point depth) — will panic if encountered.
4. `image_subscriber.rs` keeps an unused `num_messages` counter and commented-out debug logging — dead weight, safe to prune.
5. No test suite is present in-tree; `colcon test` runs empty (lcov report generated from C++ side only).
6. The package is explicitly flagged as experimental / memory-unsafe in the README; new code that calls into `rclrs`/`opencv` unsafe APIs should keep the "use at your own risk" warning truthful.
