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

pub mod camera;
pub mod viewer;

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use log::{info, warn};
use opencv::core::Mat;
use opencv::imgcodecs;
use opencv::prelude::*;
use opencv::videoio::{self, VideoCapture, VideoCaptureAPIs};

// Generate ROS 2 message definitions for sensor_msgs and std_msgs at compile time
roslibrust_codegen_macro::generate_ros_types_with_env!(
    "/opt/ros/jazzy/share/std_msgs",
    "/opt/ros/jazzy/share/builtin_interfaces",
    "/opt/ros/jazzy/share/sensor_msgs"
);

pub use builtin_interfaces::Time;
pub use sensor_msgs::Image as SensorImage;
pub use std_msgs::Header;

pub const DEFAULT_FPS: u32 = 24;
pub const TOPIC_IMAGE_RAW: &str = "/virtual_camera/image_raw";

/// Resolve data directory across local development, Docker, and Cargo paths.
pub fn get_data_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("VCAM_DATA_DIR") {
        let p = PathBuf::from(dir);
        if let Ok(canon) = fs::canonicalize(&p) {
            return canon;
        }
        return p;
    }
    let local_data = PathBuf::from("data");
    if let Ok(canon) = fs::canonicalize(&local_data) {
        return canon;
    }
    let ws_data = PathBuf::from("/virtual_camera_ws/src/virtual_camera/data");
    if let Ok(canon) = fs::canonicalize(&ws_data) {
        return canon;
    }
    let manifest_data = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data");
    if let Ok(canon) = fs::canonicalize(&manifest_data) {
        return canon;
    }
    manifest_data
}

/// Abstract media source streaming either looped video or static image.
pub enum MediaSource {
    Video {
        cap: VideoCapture,
        path: PathBuf,
    },
    Image {
        img: Mat,
    },
}

impl MediaSource {
    /// Initialize media source following the C++ VirtualCamera fallback hierarchy:
    /// 1. Try opening data/input_data as video.
    /// 2. If video fails, try loading data/input_data as static image.
    /// 3. If image fails, fall back to data/nadezhda_diskant.jpg.
    pub fn new(data_dir: &Path) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let input_path = data_dir.join("input_data");
        let default_image_path = data_dir.join("nadezhda_diskant.jpg");

        info!("Target media path: {:?}", input_path);

        // Attempt 1: Try reading as video
        if input_path.exists() {
            if let Ok(mut cap) = VideoCapture::from_file(
                &input_path.to_string_lossy(),
                VideoCaptureAPIs::CAP_ANY as i32,
            ) {
                if cap.is_opened().unwrap_or(false) {
                    let mut test_frame = Mat::default();
                    if cap.read(&mut test_frame).unwrap_or(false) && !test_frame.empty() {
                        let _ = cap.set(videoio::CAP_PROP_POS_FRAMES, 0.0);
                        info!("Successfully loaded video from {:?}", input_path);
                        return Ok(MediaSource::Video {
                            cap,
                            path: input_path,
                        });
                    }
                }
            }
            warn!("Error opening video file. Opening as image instead.");

            // Attempt 2: Try reading as static image
            if let Ok(img) = imgcodecs::imread(&input_path.to_string_lossy(), imgcodecs::IMREAD_COLOR) {
                if !img.empty() {
                    info!("Successfully loaded static image from {:?}", input_path);
                    return Ok(MediaSource::Image { img });
                }
            }
        }

        // Attempt 3: Fall back to default test image
        warn!(
            "[{:?}] - IMAGE EMPTY or NOT FOUND. Assigning DEFAULT_IMAGE: {:?}",
            input_path, default_image_path
        );
        let default_img = imgcodecs::imread(
            &default_image_path.to_string_lossy(),
            imgcodecs::IMREAD_COLOR,
        )?;
        if default_img.empty() {
            return Err(format!(
                "Failed to load default fallback image from {:?}",
                default_image_path
            )
            .into());
        }

        info!("Successfully loaded fallback image from {:?}", default_image_path);
        Ok(MediaSource::Image { img: default_img })
    }

    /// Retrieve the next frame. For video, reaching EOF re-opens the file to loop continuously.
    pub fn get_frame(&mut self) -> Result<Mat, Box<dyn std::error::Error + Send + Sync>> {
        match self {
            MediaSource::Video { cap, path } => {
                let mut frame = Mat::default();
                let success = cap.read(&mut frame).unwrap_or(false);

                if !success || frame.empty() {
                    // Loop back to beginning
                    let _ = cap.open_file(&path.to_string_lossy(), VideoCaptureAPIs::CAP_ANY as i32);
                    let retry_success = cap.read(&mut frame).unwrap_or(false);
                    if !retry_success || frame.empty() {
                        return Err("Failed to reload video frame after reaching EOF".into());
                    }
                }
                Ok(frame)
            }
            MediaSource::Image { img } => Ok(img.clone()),
        }
    }
}

/// Convert OpenCV cv::Mat into ROS 2 sensor_msgs/msg/Image (bgr8 encoding).
pub fn mat_to_sensor_msgs_image(
    mat: &Mat,
    frame_id: &str,
) -> Result<SensorImage, Box<dyn std::error::Error + Send + Sync>> {
    let height = mat.rows() as u32;
    let width = mat.cols() as u32;
    let channels = mat.channels() as u32;
    let step = width * channels;

    let now = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;
    let sec = now.as_secs() as i32;
    let nanosec = now.subsec_nanos();

    let header = Header {
        stamp: Time { sec, nanosec },
        frame_id: frame_id.to_string(),
    };

    let data_bytes = mat.data_bytes()?;

    Ok(SensorImage {
        header,
        height,
        width,
        encoding: "bgr8".to_string(),
        is_bigendian: 0,
        step,
        data: data_bytes.to_vec(),
    })
}

/// Advance the terminal progress cursor ('/', '-', '\', '|').
pub fn advance_cursor(pos: &mut usize) {
    const CURSOR: [char; 4] = ['/', '-', '\\', '|'];
    print!("\rPublishing {}\x08", CURSOR[*pos]);
    let _ = std::io::stdout().flush();
    *pos = (*pos + 1) % 4;
}
