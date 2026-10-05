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
Cargo.toml        Rust package manifest; one [lib] + two [[bin]] targets (both `required-features=["ros"]`)
IMPLEMENTATION.md Test-suite design doc (strategy, coverage table, status, verify commands)
src/
  lib.rs               `pub mod logic;` (crate doc: layout + how to run the std-only test suite)
  logic.rs             std-only shared logic (FileKind, MatSpec, encoding/header/magic-byte helpers) + 24 tests
  image_publisher.rs   Publisher node: reads data/input_data (video or image), publishes Image  [gated: ros]
  image_subscriber.rs  Subscriber node: receives Image, displays via OpenCV highgui            [gated: ros]
package.xml       ament package metadata (ROS name: virtual_camera, version 0.0.0)
launch/
  run.launch.py   Launches image_publisher; conditionally image_subscriber
scripts/          Bash helpers (build, run, docker, coverage)
Dockerfile        Full in-container build of ros2_rust + this package (user "user", workdir /workspace)
data/             Input media + the `input_data` symlink (git-ignored; publisher reads data/input_data)
build/ install/ log/ target/   colcon/cargo artifacts (git-ignored)
CONTRIBUTING.md   Apache-2 license terms + DCO sign-off requirement
```

## Key As-Built Facts (read before editing)

### Binaries & topics

| Binary | Node name | Role |
|---|---|---|
| `image_publisher` | `image_publisher` | Publisher on `/virtual_camera/image_raw` (hardcoded) |
| `image_subscriber` | `image_subscriber` | Subscriber on `/virtual_camera/image_raw` (hardcoded), shows `Image Subscriber` HWindow |

### Crate layout & test strategy (feature-gated, two-tier)

- **Cargo feature `ros` (default-on)** gates `rclrs`, `std_msgs`, `sensor_msgs`, `builtin_interfaces`, and `opencv`. Both `[[bin]]` targets declare `required-features=["ros"]`, so a build *without* the `ros` feature fails to build the bins but still builds the `virtual_camera` lib.
- **`src/logic.rs` is the pure, std-only core** (magic-byte sniffing, `MatSpec`/encoding lookups, `image_header_fields` header math, `detect_file_kind`). It holds **24 unit tests** and no native deps. The logic is duplicated conceptually in the two bins but the bins are thin ROS/OpenCV glue around it.
- The ROS message crates and `rclrs` are **not on crates.io** — they only resolve from a built `ros2_rust` workspace. So a bare `cargo build`/`cargo test` on a clean machine cannot resolve them regardless of features.
- **Two verification tiers:**
  1. **std-only (anywhere, bare Rust toolchain):** copy `src/logic.rs` into a scratch lib crate and run `cargo test` — the documented *probe* pattern in `IMPLEMENTATION.md`. This is the fast, dependency-free loop for the pure logic + tests.
  2. **full package (needs `ros2_rust` workspace):** `colcon build` then `colcon test --packages-select virtual_camera` — validates the ROS/OpenCV bins that the std-only suite does *not* cover.
- Bin code (publisher / subscriber) is **not** covered by the std-only suite; see `IMPLEMENTATION.md` "Remaining risks / notes".

### The 24 std-only tests (what they cover)

All live in `src/logic.rs` (`#[cfg(test)] mod tests`), exercise the std-only core, and run with **no ROS/OpenCV** — a bare Rust toolchain suffices. 24 tests across 6 areas:

| Area | Tests (n) | What they assert |
|---|---|---|
| **Magic-byte classification** | 7 | JPEG/PNG/GIF → `FileKind::Image`; MP4 `ftyp`+`mp42`, MPEG PS/VS (`00 00 01 BA/B3`), `OggS`, EBML/MKV → `Video`; near-miss prefixes (`FF D8 00`, `ftypisom`, truncated `FF D8`) rejected; garbage + empty header → `Unknown`. Covers `matches_image_magic`, `matches_video_magic`, `classify_magic_bytes`. |
| **`detect_file_kind` (real filesystem)** | 5 | Real temp JPEG file → Image; real temp MP4 file → Video; empty file → Unknown; missing file → `io::Error`; a directory → Unknown. |
| **`subscriber_mat_spec`** | 4 | All 6 encodings map to a `MatSpec`; `rgb8`/`rgba8` set `convert_rgb` (BGR/mono do not); `cv_type` decodes to correct OpenCV `depth` (bits 0–2) + `channels` (`(>>3)+1`) for all mappings; 7 bad encodings (empty, `mono32`, `bgr16`, `yuv420p`, `BGRA8`, …) → `None`. |
| **`publisher_encoding`** | 3 | The 4 publishable combos → correct labels (`mono8`/`bgr8`/`bgra8`/`mono16`); 0/2/5-channel, 16-bit BGR, and any `CV_32F`/`CV_64F` → `None` (no panic). |
| **`image_header_fields` (pure math)** | 2 | `step == width × channels` for 6×(3,4) shapes; bad depth → `None`; zero-height still computes `step` (returns `(0, 3, 9, "bgr8")`). |
| **Round-trip consistency** | 1 | Every label the publisher emits is accepted by `subscriber_mat_spec`; all `SUPPORTED_ENCODINGS` are non-empty ASCII-alnum strings. |

**Not covered** (out of scope for std-only): the two ROS/OpenCV bins (`image_publisher.rs`, `image_subscriber.rs`) — needs a `ros2_rust` workspace; see "Remaining risks" in `IMPLEMENTATION.md`.

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

- `README.md` shows CI build (industrial_ci) and license badges; the workflow files live on the remote (`.github/` is not present in this checkout).
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

Run the **std-only logic test suite** (no ROS/OpenCV needed) via the probe pattern
from `IMPLEMENTATION.md`:

```bash
# Throwaway scratch crate compiles src/logic.rs and runs its 24 tests:
cp src/logic.rs /tmp/vcam_probe/logic.rs
cd /tmp/vcam_probe && cargo test

# Full package (requires a ros2_rust workspace where the msg crates resolve):
colcon build
colcon test --packages-select virtual_camera
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
5. The std-only test suite in `src/logic.rs` (24 tests) covers the pure logic (magic bytes, encoding/header lookups, file-kind detection) but **not** the ROS/OpenCV bins; `colcon test` still exercises only what the bins reach. See "Crate layout & test strategy" above for the two-tier verification flow.
6. The package is explicitly flagged as experimental / memory-unsafe in the README; new code that calls into `rclrs`/`opencv` unsafe APIs should keep the "use at your own risk" warning truthful.
