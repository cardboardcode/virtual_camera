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

use std::sync::atomic::Ordering;
use opencv::core::Mat;
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
