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
use virtual_camera::viewer::ImageViewer;

#[derive(Parser, Debug)]
#[command(author, version, about = "Standalone image viewer for virtual_camera", long_about = None)]
struct Args {
    /// Rosbridge WebSocket URL
    #[arg(long, default_value = "ws://localhost:9090")]
    ws_url: String,

    /// Run viewer in headless mode (no GUI window displayed)
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

    info!("Starting standalone image_viewer (Rust)...");
    info!("rosbridge URL: {}", args.ws_url);

    let mut viewer = ImageViewer::new(args.headless);
    let stop_handle = viewer.get_stop_handle();

    tokio::spawn(async move {
        if let Ok(()) = tokio::signal::ctrl_c().await {
            info!("Received shutdown signal. Stopping image_viewer...");
            stop_handle.store(false, Ordering::Relaxed);
        }
    });

    if let Err(e) = viewer.run(&args.ws_url).await {
        log::error!("ImageViewer encountered error: {}", e);
    }

    info!("image_viewer shut down gracefully.");
    Ok(())
}
