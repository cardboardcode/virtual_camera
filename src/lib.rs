//! `virtual_camera` — a ROS 2 Humble package that simulates a camera by
//! replaying a static video or image file as a `sensor_msgs/Image` topic.
//!
//! # Layout
//!
//! | target            | path                          | gated by |
//! |-------------------|-------------------------------|----------|
//! | lib (pure logic)  | `src/lib.rs` → `src/logic.rs` | —        |
//! | `image_publisher` | `src/image_publisher.rs`      | `ros`    |
//! | `image_subscriber`| `src/image_subscriber.rs`     | `ros`    |
//!
//! The library module (`virtual_camera::logic`) is std-only: it holds the
//! magic-byte file sniffing logic, the encoder/decoder encoding lookups, and
//! the header field math that both nodes share.  Because it has no
//! `rclrs`/`opencv` dependency, the whole test suite can be run under
//!
//! ```bash
//! cargo test --no-default-features
//! ```
//!
//! without a ROS 2 Humble toolchain, colcon, or OpenCV installed.
//!
//! The two binaries (`src/image_publisher.rs`, `src/image_subscriber.rs`)
//! are gated behind the `ros` cargo feature (default-on) so the above
//! command works on bare Rust installs; `colcon build` / `cargo build`
//! (defaults) still produce both executables exactly as before.
#![allow(clippy::unwrap_used)]

pub mod logic;
