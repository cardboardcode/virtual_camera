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

use opencv::core::Mat;
use std::sync::atomic::Ordering;
use virtual_camera::viewer::ImageViewer;

#[test]
fn test_image_viewer_headless() {
    let viewer = ImageViewer::new(true);
    let stop_handle = viewer.get_stop_handle();
    assert!(stop_handle.load(Ordering::Relaxed));
}

#[test]
fn test_image_viewer_show_mat_headless() {
    let viewer = ImageViewer::new(true);
    let mat = Mat::default();
    assert!(viewer.show_mat(&mat).is_ok());
}

#[test]
fn test_image_viewer_with_display() {
    // If DISPLAY is set (e.g. under xvfb-run or X11 session), test GUI initialization
    let is_headless = std::env::var("DISPLAY").is_err();
    let viewer = ImageViewer::new(is_headless);
    let stop_handle = viewer.get_stop_handle();
    assert!(stop_handle.load(Ordering::Relaxed));
}

#[test]
fn test_decode_image_rejects_invalid_geometry() {
    fn make_image(
        width: u32,
        height: u32,
        step: u32,
        encoding: &str,
        data: Vec<u8>,
    ) -> virtual_camera::SensorImage {
        virtual_camera::SensorImage {
            header: virtual_camera::Header {
                stamp: virtual_camera::Time { sec: 0, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            height,
            width,
            encoding: encoding.to_string(),
            is_bigendian: 0,
            step,
            data,
        }
    }

    // step == 0
    let msg = make_image(8, 8, 0, "bgr8", vec![0u8; 16]);
    assert!(ImageViewer::decode_image(&msg).is_err());

    // width == 0
    let msg = make_image(0, 8, 16, "bgr8", vec![0u8; 16]);
    assert!(ImageViewer::decode_image(&msg).is_err());

    // height == 0
    let msg = make_image(8, 0, 16, "bgr8", vec![0u8; 16]);
    assert!(ImageViewer::decode_image(&msg).is_err());

    // height == u32::MAX: i32 truncation case
    let msg = make_image(1, u32::MAX, 16, "bgr8", vec![0u8; 16]);
    assert!(ImageViewer::decode_image(&msg).is_err());

    // data too short for step * height (with valid non-zero dims)
    let msg = make_image(4, 4, 12, "bgr8", vec![0u8; 1]);
    assert!(ImageViewer::decode_image(&msg).is_err());

    // unsupported encoding
    let msg = make_image(4, 4, 12, "junk_encoding", vec![0xFFu8; 12]);
    assert!(ImageViewer::decode_image(&msg).is_err());
}

#[test]
fn test_decode_image_accepts_valid_small_image() {
    // 2x2 bgr8: step = 2 * 3 = 6, data = 12 bytes
    let msg = virtual_camera::SensorImage {
        header: virtual_camera::Header {
            stamp: virtual_camera::Time { sec: 0, nanosec: 0 },
            frame_id: "world".to_string(),
        },
        height: 2,
        width: 2,
        encoding: "bgr8".to_string(),
        is_bigendian: 0,
        step: 6,
        data: vec![0u8; 12],
    };
    assert!(ImageViewer::decode_image(&msg).is_ok());
}
