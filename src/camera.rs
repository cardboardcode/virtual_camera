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

use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;

use log::{debug, error, info};
use opencv::core::Mat;
use roslibrust::rosbridge::{ClientHandle, Publisher};

use crate::{
    advance_cursor, frame_interval, mat_to_sensor_msgs_image, MediaSource, SensorImage,
    TOPIC_IMAGE_RAW,
};

pub struct VirtualCamera {
    media_source: MediaSource,
    fps: Arc<AtomicU32>,
    cursor_pos: usize,
    publisher: Publisher<SensorImage>,
    running: Arc<AtomicBool>,
    frame_sender: Option<tokio::sync::mpsc::Sender<Mat>>,
}

impl VirtualCamera {
    /// Create and connect a new VirtualCamera node via roslibrust client handle.
    pub async fn new(
        ws_url: &str,
        fps: u32,
        data_dir: &Path,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        info!("Connecting to rosbridge server at {}...", ws_url);
        let client = ClientHandle::new(ws_url).await?;
        info!("Connected to rosbridge successfully.");

        info!("Advertising topic: {}", TOPIC_IMAGE_RAW);
        let publisher = client.advertise::<SensorImage>(TOPIC_IMAGE_RAW).await?;

        let media_source = MediaSource::new(data_dir)?;

        Ok(Self {
            media_source,
            fps: Arc::new(AtomicU32::new(fps)),
            cursor_pos: 0,
            publisher,
            running: Arc::new(AtomicBool::new(true)),
            frame_sender: None,
        })
    }

    /// Register an optional frame sender channel for in-process frame forwarding.
    pub fn set_frame_sender(&mut self, tx: tokio::sync::mpsc::Sender<Mat>) {
        self.frame_sender = Some(tx);
    }

    /// Dynamically update publishing frame rate.
    pub fn set_fps(&self, new_fps: u32) {
        if new_fps > 0 {
            info!(
                "Updating FPS from {} to {}",
                self.fps.load(Ordering::Relaxed),
                new_fps
            );
            self.fps.store(new_fps, Ordering::Relaxed);
        }
    }

    /// Get a handle to stop the publishing loop.
    pub fn get_stop_handle(&self) -> Arc<AtomicBool> {
        self.running.clone()
    }

    /// Run the main publishing loop until canceled.
    pub async fn run(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        info!(
            "Starting publication loop at {} FPS on topic {}",
            self.fps.load(Ordering::Relaxed),
            TOPIC_IMAGE_RAW
        );

        while self.running.load(Ordering::Relaxed) {
            let frame_duration = frame_interval(self.fps.load(Ordering::Relaxed));

            let frame = match self.media_source.get_frame() {
                Ok(f) => f,
                Err(e) => {
                    error!("Error retrieving frame: {}", e);
                    tokio::time::sleep(frame_duration).await;
                    continue;
                }
            };

            if let Some(ref tx) = self.frame_sender {
                let _ = tx.try_send(frame.clone());
            }

            let msg = match mat_to_sensor_msgs_image(&frame, "world") {
                Ok(m) => m,
                Err(e) => {
                    error!("Error converting frame to ROS image message: {}", e);
                    tokio::time::sleep(frame_duration).await;
                    continue;
                }
            };

            debug!("Publishing frame: {}x{}", msg.width, msg.height);
            if let Err(e) = self.publisher.publish(&msg).await {
                error!("Error publishing image: {}", e);
            }

            advance_cursor(&mut self.cursor_pos);

            tokio::time::sleep(frame_duration).await;
        }

        info!("VirtualCamera publication loop finished.");
        Ok(())
    }
}
