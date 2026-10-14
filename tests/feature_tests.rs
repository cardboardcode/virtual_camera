// Copyright 2026 Bey Hao Yun
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Feature-oriented tests for the minimal ROS 2 feature set of `virtual_camera`:
//!
//!   1. Loading `input_data` and producing the message published on the topic.
//!   2. Allowing the publishing FPS to be changed (frame-interval math).
//!   3. Running with, and running without, the attached image-viewer.
//!
//! These exercise the *unit* surface of each feature without requiring a live
//! rosbridge broker or a display. Actual topic receipt over DDS is covered by
//! `scripts/test_ros2_e2e.bash`.

use opencv::prelude::*;
use virtual_camera::viewer::ImageViewer;
use virtual_camera::{frame_interval, get_data_dir, mat_to_sensor_msgs_image, MediaSource};

/// Load media the way the node does (image, video, or fallback) and grab a frame.
fn load_a_frame() -> opencv::core::Mat {
    let data_dir = get_data_dir();
    let mut media = MediaSource::new(&data_dir)
        .expect("MediaSource should load input_data (image/video) or the fallback image");
    let frame = media
        .get_frame()
        .expect("get_frame should return a valid frame");
    assert!(!frame.empty(), "frame should not be empty");
    frame
}

// ---------------------------------------------------------------------------
// Feature 1: Loading input_data and publishing on the respective ROS 2 topic
// ---------------------------------------------------------------------------

#[test]
fn test_publishes_on_expected_topic() {
    // The node publishes `sensor_msgs/msg/Image` on this fixed topic.
    assert_eq!(virtual_camera::TOPIC_IMAGE_RAW, "/virtual_camera/image_raw");
    // Startup FPS default surfaced to the launch file / CLI.
    assert_eq!(virtual_camera::DEFAULT_FPS, 24);
}

#[test]
fn test_input_data_produces_publishable_message() {
    let frame = load_a_frame();

    let msg = mat_to_sensor_msgs_image(&frame, "world")
        .expect("frame should convert into a publishable sensor_msgs/Image");

    assert_eq!(msg.header.frame_id, "world");
    assert_eq!(msg.encoding, "bgr8");
    assert_eq!(msg.width, frame.cols() as u32);
    assert_eq!(msg.height, frame.rows() as u32);
    assert_eq!(msg.step, (frame.cols() * 3) as u32);
    assert_eq!(msg.data.len(), (msg.height * msg.step) as usize);
    assert!(!msg.data.is_empty(), "message must carry pixel data");
}

// ---------------------------------------------------------------------------
// Feature 2: Allowing FPS changes (inter-frame interval must track the rate)
// ---------------------------------------------------------------------------

#[test]
fn test_fps_change_updates_frame_interval() {
    // Raising the FPS must shorten the inter-frame delay (monotonic non-increasing).
    let fps_5 = frame_interval(5);
    let fps_12 = frame_interval(12);
    let fps_24 = frame_interval(24);
    let fps_30 = frame_interval(30);
    assert!(fps_5 >= fps_12 && fps_12 >= fps_24 && fps_24 >= fps_30);
    assert!(
        fps_5 > fps_30,
        "5 FPS interval must be strictly larger than 30 FPS interval"
    );

    // A runtime change from 24 -> 48 FPS must shorten the interval.
    let before = frame_interval(24);
    let after = frame_interval(48);
    assert!(
        after < before,
        "raising 24 -> 48 FPS must shorten the frame interval"
    );

    // Lowering the FPS must lengthen the interval.
    let at_high = frame_interval(60);
    let at_low = frame_interval(15);
    assert!(
        at_low > at_high,
        "lowering the FPS must lengthen the frame interval"
    );
}

#[test]
fn test_fps_change_clamped_for_non_positive_rate() {
    // The loop guards against divide-by-zero: 0 FPS behaves like 1 FPS.
    assert_eq!(frame_interval(0), frame_interval(1));
    assert!(
        frame_interval(0) > std::time::Duration::ZERO,
        "interval must stay positive"
    );
}

// ---------------------------------------------------------------------------
// Feature 3: Running with, and without, the attached image-viewer
// ---------------------------------------------------------------------------

#[test]
fn test_run_without_image_viewer_pipeline() {
    // `use_image_viewer = false`: the node simply converts each frame to a
    // publishable message and publishes it (no viewer decode/show involved).
    for _ in 0..3 {
        let frame = load_a_frame();
        let msg = mat_to_sensor_msgs_image(&frame, "world")
            .expect("frame should convert without a viewer attached");
        assert!(
            !msg.data.is_empty(),
            "published message must carry pixel data"
        );
        assert_eq!(msg.encoding, "bgr8");
    }
}

#[test]
fn test_run_with_image_viewer_roundtrip() {
    // `use_image_viewer = true`: the viewer receives the published message,
    // decodes it back to an OpenCV Mat, and (in headless mode) renders it as a
    // safe no-op that still succeeds.
    let frame = load_a_frame();
    let msg = mat_to_sensor_msgs_image(&frame, "world").expect("frame -> message");

    let decoded = ImageViewer::decode_image(&msg).expect("viewer should decode the message");
    assert!(!decoded.empty(), "decoded mat should not be empty");
    assert_eq!(decoded.rows(), msg.height as i32);
    assert_eq!(decoded.cols(), msg.width as i32);

    // Headless rendering must be a successful no-op (keeps tests CI-safe).
    let viewer = ImageViewer::new(true);
    assert!(!viewer.show_mat(&decoded).is_err());
}
