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

use std::fs;
use std::os::unix::fs::symlink;
use std::path::PathBuf;

use opencv::imgcodecs;
use opencv::prelude::*;
use virtual_camera::viewer::ImageViewer;
use virtual_camera::{advance_cursor, get_data_dir, mat_to_sensor_msgs_image, MediaSource};

fn create_isolated_test_dir(test_id: &str, target_filename: &str) -> PathBuf {
    let base_data_dir = get_data_dir();
    let temp_dir =
        std::env::temp_dir().join(format!("vcam_test_{}_{}", test_id, std::process::id()));
    let _ = fs::remove_dir_all(&temp_dir);
    fs::create_dir_all(&temp_dir).expect("Failed to create temp dir");

    let orig_default = base_data_dir.join("nadezhda_diskant.jpg");
    let test_default = temp_dir.join("nadezhda_diskant.jpg");
    fs::copy(&orig_default, &test_default).expect("Failed to copy default image");

    let orig_target = base_data_dir.join(target_filename);
    let test_input = temp_dir.join("input_data");
    let _ = symlink(
        orig_target.canonicalize().unwrap_or(orig_target),
        &test_input,
    );

    temp_dir
}

#[test]
fn test_image_input_data() {
    let test_dir = create_isolated_test_dir("image", "nadezhda_diskant.jpg");
    let mut media = MediaSource::new(&test_dir).expect("Failed to create MediaSource with image");

    let frame = media.get_frame().expect("Failed to read frame from image");
    assert!(!frame.empty(), "Frame should not be empty");
    assert!(frame.rows() > 0 && frame.cols() > 0);

    let msg =
        mat_to_sensor_msgs_image(&frame, "world").expect("Failed to convert Mat to Image msg");
    assert_eq!(msg.header.frame_id, "world");
    assert_eq!(msg.encoding, "bgr8");
    assert_eq!(msg.height, frame.rows() as u32);
    assert_eq!(msg.width, frame.cols() as u32);
    assert_eq!(msg.step, (frame.cols() * 3) as u32);
    assert_eq!(msg.data.len(), (msg.height * msg.step) as usize);

    let _ = fs::remove_dir_all(&test_dir);
}

#[test]
fn test_video_input_data() {
    let test_dir = create_isolated_test_dir("video", "video.mp4");
    let mut media = MediaSource::new(&test_dir).expect("Failed to create MediaSource with video");

    // Read multiple frames to verify stream reading
    for _ in 0..5 {
        let frame = media.get_frame().expect("Failed to read frame from video");
        assert!(!frame.empty(), "Video frame should not be empty");
    }

    let _ = fs::remove_dir_all(&test_dir);
}

#[test]
fn test_fallback_when_symlink_broken() {
    let base_data_dir = get_data_dir();
    let temp_dir = std::env::temp_dir().join(format!("vcam_test_broken_{}", std::process::id()));
    let _ = fs::remove_dir_all(&temp_dir);
    fs::create_dir_all(&temp_dir).expect("Failed to create temp dir");

    let orig_default = base_data_dir.join("nadezhda_diskant.jpg");
    let test_default = temp_dir.join("nadezhda_diskant.jpg");
    fs::copy(&orig_default, &test_default).expect("Failed to copy default image");

    let nonexistent = temp_dir.join("nonexistent_file_12345.mp4");
    let test_input = temp_dir.join("input_data");
    let _ = symlink(&nonexistent, &test_input);

    let mut media =
        MediaSource::new(&temp_dir).expect("MediaSource should fall back to default image");
    let frame = media
        .get_frame()
        .expect("Should retrieve frame from fallback image");
    assert!(!frame.empty(), "Fallback frame should not be empty");

    let _ = fs::remove_dir_all(&temp_dir);
}

#[test]
fn test_advance_cursor() {
    let mut pos = 0;
    advance_cursor(&mut pos);
    assert_eq!(pos, 1);
    advance_cursor(&mut pos);
    assert_eq!(pos, 2);
    advance_cursor(&mut pos);
    assert_eq!(pos, 3);
    advance_cursor(&mut pos);
    assert_eq!(pos, 0);
}

#[test]
fn test_encoding_conversions() {
    assert_eq!(ImageViewer::encoding_to_channels("bgr8").unwrap(), 3);
    assert_eq!(ImageViewer::encoding_to_channels("rgb8").unwrap(), 3);
    assert_eq!(ImageViewer::encoding_to_channels("mono8").unwrap(), 1);
    assert_eq!(ImageViewer::encoding_to_channels("mono16").unwrap(), 1);
    assert_eq!(ImageViewer::encoding_to_channels("rgba8").unwrap(), 4);
    assert_eq!(ImageViewer::encoding_to_channels("bgra8").unwrap(), 4);
    assert_eq!(ImageViewer::encoding_to_channels("32FC1").unwrap(), 1);
    assert!(ImageViewer::encoding_to_channels("unknown_fmt").is_err());
}

#[test]
fn test_decode_image() {
    let data_dir = get_data_dir();
    let img_path = data_dir.join("nadezhda_diskant.jpg");
    let original =
        imgcodecs::imread(&img_path, imgcodecs::IMREAD_COLOR).expect("Failed to load test image");

    let msg = mat_to_sensor_msgs_image(&original, "test_frame").expect("Conversion failed");
    let decoded = ImageViewer::decode_image(&msg).expect("Decoding failed");

    assert_eq!(decoded.rows(), original.rows());
    assert_eq!(decoded.cols(), original.cols());
}
