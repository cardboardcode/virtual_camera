# IMPLEMENTATION.md — Test Suite for `virtual_camera`

Plan + status. Caveman-compressed.

## Why this shape

- ROS msg crates (`std_msgs`, `sensor_msgs`, `builtin_interfaces`) are NOT on
  crates.io — only workspace-local from a `ros2_rust` build. So plain
  `cargo test` cannot compile the full package.
- Strategy: extract all pure logic into a std-only lib module (`src/logic.rs`).
  Bins keep ROS/OpenCV code. `cargo test --no-default-features` runs anywhere
  with just a Rust toolchain. Full-package verification stays under colcon.

## Architecture

```
Cargo.toml     [lib] virtual_camera (src/lib.rs)
               [[bin]] image_publisher  required-features=["ros"]
               [[bin]] image_subscriber required-features=["ros"]
               [features] default=["ros"]; ros = dep:rclrs/std_msgs/
                          sensor_msgs/builtin_interfaces/opencv (all optional)
src/lib.rs     pub mod logic;
src/logic.rs   std-only: FileKind, MatSpec, subscriber_mat_spec(),
               publisher_encoding(), image_header_fields(),
               classify_magic_bytes(), matches_image/video_magic(),
               detect_file_kind(); SUPPORTED_ENCODINGS, PUBLISHABLE_COMBOS
src/image_publisher.rs   Ros bin: node, Mat->Image, publish loop (unchanged)
src/image_subscriber.rs  Ros bin: Image->Mat, highgui (unchanged)
/test/vcam_probe/        Scratch crate [lib path="logic.rs"] to run tests
                         without a ros2_rust workspace (throwaway, not committed)
```

## Test coverage (logic.rs, ~24 tests)

| Area | Tests |
|---|---|
| Magic bytes | JPEG/PNG/GIF -> Image; MP4 ftyp, MPEG PS/VS, OggS, EBML/MKV -> Video; near-miss prefixes rejected; garbage -> Unknown |
| detect_file_kind | real temp image file, real temp MP4 file, empty file, missing file (io::Error), directory |
| subscriber_mat_spec | all 6 encodings map; rgb*/rgba* flag convert_rgb; cv_type channel+depth decode; 7 bad encodings rejected |
| publisher_encoding | 4 publishable combos give right labels; 0/2/5-ch, 16-bit BGR, 32F/64F -> None |
| image_header_fields | step == width*channels for 3 shapes; bad depth -> None; zero-height math |
| round-trip | publisher combo set == subscriber encoding set (consistency) |

## Status

- [x] Cargo.toml feature-gating + [lib] target
- [x] lib.rs, logic.rs extracted, test module written
- [x] Fixed: `b"ftypmp42"` type error (drop local, pass literal — autoderef)
- [x] Fixed: `#[allow(dead_code)]` on CV_32F/CV_64F
- [x] Fixed: `CV_16UC1` constant — Rust `+` binds tighter than `<<`;
      `CV_16U + (0 << 3)` form now, comment added
- [x] Fixed: channel decode — OpenCV channel field is bits 4-7 (4-bit),
      `channels = ((cv_type >> 4) & 0xF) + 1`; depth = `cv_type & 0x7`
- [x] Fixed: MP4 temp-file test — real layout `[u32 BE][ftyp][mp42]`
- [x] Fixed: zero-height step expectation (9, not 0)
- [x] 24/24 green in probe run (all tests pass, `cargo clippy` clean)
- [x] Verified std-only shape compiles (`cargo check` + `cargo test` in probe)
- [x] Update AGENTS.md: lib+bin layout, feature-gating, test command,
      logic-module location, rewrite "no test suite" limitation (#5)
- [ ] Commit (emoji subject, -s sign-off, no Co-Authored-By) + push

## Remaining risks / notes

- `mp4_ftyp` logic checks bytes 4+ of an MP4 box but ALSO accepts the
  ftyp brand at the start (`starts_with(..., 8 bytes "ftypmp42")`) — mirrors
  image_publisher.rs exactly; parity over correctness. Documented, kept.
- Bin code (Ros/OpenCV) untested — needs ros2_rust workspace. Out of scope
  for std-only suite; note in AGENTS.md.
- Do not commit `build/ install/ log/ lcov/ data/ /target` (gitignored).

## Verify commands

```bash
cp src/logic.rs /tmp/vcam_probe/logic.rs
cd /tmp/vcam_probe && cargo test          # std-only suite
# full package (needs ros2_rust workspace):
colcon build
colcon test --packages-select virtual_camera
```
