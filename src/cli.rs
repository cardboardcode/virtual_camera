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

//! Command-line interface definition for the `virtual_camera` node.
//!
//! Defined in the library (rather than the binary) so that the CLI surface —
//! in particular the FPS override (`--fps`) and the viewer toggle
//! (`--use-image-viewer`) — can be exercised by unit tests without a live
//! rosbridge broker or a display.

use clap::Parser;

use crate::DEFAULT_FPS;

/// Arguments accepted by the `virtual_camera` node.
#[derive(Parser, Debug, Clone, PartialEq, Eq)]
#[command(author, version, about = "ROS 2 Virtual Camera node written in Rust", long_about = None)]
pub struct CliArgs {
    /// Publishing frame rate in FPS
    #[arg(short, long, default_value_t = DEFAULT_FPS)]
    pub fps: u32,

    /// Rosbridge WebSocket URL
    #[arg(long, default_value = "ws://localhost:9090")]
    pub ws_url: String,

    /// Attach in-process ImageViewer window
    #[arg(long, default_value_t = false)]
    pub use_image_viewer: bool,

    /// Run viewer in headless mode
    #[arg(long, default_value_t = false)]
    pub headless: bool,

    /// Enable debug logging
    #[arg(long, default_value_t = false)]
    pub debug: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_defaults() {
        let args = CliArgs::parse_from(["virtual_camera"]);
        assert_eq!(args.fps, DEFAULT_FPS);
        assert_eq!(args.ws_url, "ws://localhost:9090");
        assert!(!args.use_image_viewer);
        assert!(!args.headless);
        assert!(!args.debug);
    }

    #[test]
    fn test_cli_fps_override() {
        let args = CliArgs::parse_from(["virtual_camera", "--fps", "120"]);
        assert_eq!(args.fps, 120);
    }

    #[test]
    fn test_cli_use_image_viewer_toggle() {
        // Default: viewer detached.
        let without = CliArgs::parse_from(["virtual_camera"]);
        assert!(!without.use_image_viewer);

        // Explicitly enabled.
        let with = CliArgs::parse_from(["virtual_camera", "--use-image-viewer"]);
        assert!(with.use_image_viewer);
        // Enabling it must not disturb the other defaults.
        assert_eq!(with.fps, DEFAULT_FPS);
    }

    #[test]
    fn test_cli_rejects_non_numeric_fps() {
        let result = CliArgs::try_parse_from(["virtual_camera", "--fps", "abc"]);
        assert!(result.is_err(), "`--fps` must reject non-numeric values");
    }
}
