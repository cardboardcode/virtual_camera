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

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use log::{info, warn};
use opencv::core::Mat;
use opencv::highgui;
use opencv::imgproc;
use opencv::prelude::*;
use roslibrust::rosbridge::ClientHandle;

use crate::{SensorImage, TOPIC_IMAGE_RAW};

pub struct ImageViewer {
    headless: bool,
    running: Arc<AtomicBool>,
}

impl ImageViewer {
    pub fn new(headless: bool) -> Self {
        if !headless {
            // Attempt to create window if display is available
            let _ = highgui::named_window("image_viewer", highgui::WINDOW_AUTOSIZE);
            let _ = highgui::move_window("image_viewer", 0, 0);
            let _ = highgui::wait_key(1);
        }
        Self {
            headless,
            running: Arc::new(AtomicBool::new(true)),
        }
    }

    pub fn get_stop_handle(&self) -> Arc<AtomicBool> {
        self.running.clone()
    }

    /// Display an OpenCV Mat in the GUI window if not in headless mode.
    pub fn show_mat(&self, mat: &Mat) -> Result<(), opencv::Error> {
        if !self.headless && !mat.empty() {
            highgui::imshow("image_viewer", mat)?;
            let key = highgui::wait_key(1)?;
            if key == 27 || key == 113 {
                self.running.store(false, Ordering::Relaxed);
            }
        }
        Ok(())
    }

    /// Map ROS image encoding string to number of channels.
    pub fn encoding_to_channels(encoding: &str) -> Result<i32, String> {
        match encoding {
            "mono8" | "mono16" | "32FC1" => Ok(1),
            "bgr8" | "rgb8" => Ok(3),
            "rgba8" | "bgra8" => Ok(4),
            other => Err(format!("Unsupported encoding: {}", other)),
        }
    }

    /// Decode ROS 2 SensorImage into OpenCV cv::Mat.
    pub fn decode_image(
        msg: &SensorImage,
    ) -> Result<Mat, Box<dyn std::error::Error + Send + Sync>> {
        let channels = Self::encoding_to_channels(&msg.encoding)
            .map_err(|e| -> Box<dyn std::error::Error + Send + Sync> { e.into() })?;

        let flat = Mat::new_rows_cols_with_data(msg.height as i32, msg.step as i32, &msg.data)?;
        let reshaped = flat.reshape(channels, msg.height as i32)?;
        let mut owned = reshaped.try_clone()?;

        if msg.encoding == "rgb8" {
            let mut bgr = Mat::default();
            imgproc::cvt_color(&owned, &mut bgr, imgproc::COLOR_RGB2BGR, 0)?;
            owned = bgr;
        }

        Ok(owned)
    }

    /// Run the subscriber loop.
    pub async fn run(
        &mut self,
        ws_url: &str,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        info!(
            "ImageViewer connecting to rosbridge server at {}...",
            ws_url
        );
        let client = ClientHandle::new(ws_url).await?;
        info!("Subscribing to topic: {}", TOPIC_IMAGE_RAW);

        let subscriber = client.subscribe::<SensorImage>(TOPIC_IMAGE_RAW).await?;
        info!("ImageViewer subscription active.");

        while self.running.load(Ordering::Relaxed) {
            tokio::select! {
                biased;
                msg = subscriber.next() => {
                    match Self::decode_image(&msg) {
                        Ok(mat) => {
                            let _ = self.show_mat(&mat);
                        }
                        Err(e) => {
                            warn!("Failed to decode received image: {}", e);
                        }
                    }
                }
                _ = tokio::time::sleep(std::time::Duration::from_millis(500)), if !self.running.load(Ordering::Relaxed) => {
                    break;
                }
            }
        }

        info!("ImageViewer subscriber loop ended.");
        Ok(())
    }
}
