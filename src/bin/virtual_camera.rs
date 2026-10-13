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

use clap::Parser;
use log::info;
use virtual_camera::camera::VirtualCamera;
use virtual_camera::viewer::ImageViewer;
use virtual_camera::{get_data_dir, DEFAULT_FPS};

#[derive(Parser, Debug)]
#[command(author, version, about = "ROS 2 Virtual Camera node written in Rust", long_about = None)]
struct Args {
    /// Publishing frame rate in FPS
    #[arg(short, long, default_value_t = DEFAULT_FPS)]
    fps: u32,

    /// Rosbridge WebSocket URL
    #[arg(long, default_value = "ws://localhost:9090")]
    ws_url: String,

    /// Attach in-process ImageViewer window
    #[arg(long, default_value_t = false)]
    use_image_viewer: bool,

    /// Run viewer in headless mode
    #[arg(long, default_value_t = false)]
    headless: bool,

    /// Enable debug logging
    #[arg(long, default_value_t = false)]
    debug: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let args = Args::parse();

    let log_level = if args.debug {
        log::LevelFilter::Debug
    } else {
        log::LevelFilter::Info
    };
    env_logger::Builder::from_default_env()
        .filter_level(log_level)
        .init();

    info!("Starting virtual_camera node (Rust)...");
    info!("FPS target: {}", args.fps);
    info!("rosbridge URL: {}", args.ws_url);
    info!("use_image_viewer: {}", args.use_image_viewer);

    let data_dir = get_data_dir();
    info!("Resolved data directory: {:?}", data_dir);

    // Initialize publisher node
    let mut camera = VirtualCamera::new(&args.ws_url, args.fps, &data_dir).await?;
    let stop_cam = camera.get_stop_handle();

    // Listen for Ctrl-C signal
    let stop_handle = stop_cam.clone();
    tokio::spawn(async move {
        if let Ok(()) = tokio::signal::ctrl_c().await {
            info!("Received shutdown signal. Stopping virtual_camera...");
            stop_handle.store(false, Ordering::Relaxed);
        }
    });

    if args.use_image_viewer {
        info!("Starting in-process ImageViewer on main thread...");
        let (frame_tx, mut frame_rx) = tokio::sync::mpsc::channel::<opencv::core::Mat>(2);
        camera.set_frame_sender(frame_tx);

        let viewer = ImageViewer::new(args.headless);
        let viewer_stop = viewer.get_stop_handle();

        // Spawn camera publication loop in Tokio task
        let cam_stop = stop_cam.clone();
        let cam_task = tokio::spawn(async move {
            if let Err(e) = camera.run().await {
                log::error!("VirtualCamera encountered error: {}", e);
            }
        });

        // Run viewer event and display loop on the OS main thread
        while cam_stop.load(Ordering::Relaxed) && viewer_stop.load(Ordering::Relaxed) {
            match frame_rx.recv().await {
                Some(frame) => {
                    let _ = viewer.show_mat(&frame);
                }
                None => break,
            }
        }

        cam_stop.store(false, Ordering::Relaxed);
        let _ = cam_task.await;
    } else {
        // Run publisher loop directly on main thread
        if let Err(e) = camera.run().await {
            log::error!("VirtualCamera encountered error: {}", e);
        }
    }

    info!("virtual_camera shut down gracefully.");
    Ok(())
}
