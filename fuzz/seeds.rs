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

//! Seed corpus generator for the `decode_image` fuzz target.
//!
//! Emits 6 files into `fuzz/seed_corpus/decode_image/` using the exact input
//! layout the target deserializes:
//!
//!     u32 LE width ‖ u32 LE height ‖ u32 LE step ‖ u32 LE encoding_index ‖ payload
//!
//! encoding_index selects from the target's ENCODINGS array
//! (0="mono8", 1="bgr8", 2="rgb8", 3="mono16", 4="rgba8", 5="bgra8",
//!  6="32FC1", 7="junk_encoding").
//!
//! The generated files are build artifacts (see .gitignore); do not commit
//! them. Every consumer (local runs, `scripts/*`, CI in `.github/workflows/
//! fuzz.yaml`) must invoke `cargo run --bin seeds` before `cargo fuzz run`.

use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

use opencv::prelude::*;

fn seed(width: u32, height: u32, step: u32, enc_idx: u32, payload: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(16 + payload.len());
    buf.extend_from_slice(&width.to_le_bytes());
    buf.extend_from_slice(&height.to_le_bytes());
    buf.extend_from_slice(&step.to_le_bytes());
    buf.extend_from_slice(&enc_idx.to_le_bytes());
    buf.extend_from_slice(payload);
    buf
}

fn valid_bgr8() -> (u32, u32, u32, Vec<u8>) {
    // Prefer the shipped JPEG; fall back to a 16x16 zero-filled bgr8 frame.
    // The binary may run from the fuzz crate dir (CI/Docker layout: /virt/fuzz)
    // or the repo root, so try several known locations.
    let mut candidates = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        candidates.push(cwd.join("data/nadezhda_diskant.jpg"));
    }
    if let Ok(manifest) = std::env::var("CARGO_MANIFEST_DIR") {
        candidates.push(PathBuf::from(manifest).join("../data/nadezhda_diskant.jpg"));
    }
    candidates.push(PathBuf::from("/virt/data/nadezhda_diskant.jpg"));

    let mut mat = opencv::core::Mat::default();
    for cand in &candidates {
        match opencv::imgcodecs::imread(
            &cand,
            opencv::imgcodecs::IMREAD_COLOR,
        ) {
            Ok(m) if !m.empty() => {
                mat = m;
                break;
            }
            _ => {}
        }
    }
    match virtual_camera::mat_to_sensor_msgs_image(&mat, "world") {
        Ok(msg) if msg.data.len() > 16 => (msg.width, msg.height, msg.step, msg.data),
        _ => fallback(),
    }
}

fn fallback() -> (u32, u32, u32, Vec<u8>) {
    (16, 16, 48, vec![0u8; 2304])
}

fn main() -> ExitCode {
    let dir = PathBuf::from("fuzz/seed_corpus/decode_image");
    fs::create_dir_all(&dir).expect("failed to create seed corpus dir");

    let (w, h, s, data) = valid_bgr8();
    let seeds: [(&str, Vec<u8>); 6] = [
        ("01_valid_bgr8.bin", seed(w, h, s, 1, &data)),
        (
            "02_height_max_u32.bin",
            seed(1, 0xFFFFFFFF, 16, 1, &[0u8; 16]),
        ),
        (
            "03_data_truncated.bin",
            seed(8, 8, 24, 1, &[0xAB; 256]),
        ),
        ("04_step_zero.bin", seed(8, 8, 0, 1, &[0u8; 16])),
        ("05_width_zero.bin", seed(0, 8, 16, 1, &[0u8; 16])),
        ("06_junk_encoding.bin", seed(4, 4, 12, 7, &[0xFF; 12])),
    ];

    for (name, bytes) in &seeds {
        let path = dir.join(name);
        fs::write(&path, bytes).expect("failed to write seed file");
        println!("wrote {} ({} bytes)", path.display(), bytes.len());
    }

    // Guard: every committed-era seed name must exist and carry a header
    // payload (16-byte header + >= 1 byte of payload), i.e. it must have
    // been regenerated rather than left as an empty/stale artifact.
    for (name, expected) in expected_sizes(&data) {
        let path = dir.join(name);
        let len = fs::metadata(&path)
            .map(|m| m.len())
            .unwrap_or_else(|e| panic!("seed {path:?} missing after generation: {e}"));
        assert!(
            len == expected,
            "seed {path:?} size {len} != expected {expected} (stale from a prior generation?); rerun 'cargo run --bin seeds'"
        );
    }

    ExitCode::SUCCESS
}

/// Expected on-disk sizes (in bytes) of the generated seeds, in order.
/// Used by the post-write guard so a stale/incomplete seed corpus fails fast.
fn expected_sizes(payload: &[u8]) -> Vec<(&'static str, u64)> {
    vec![
        ("01_valid_bgr8.bin", 16 + payload.len() as u64),
        ("02_height_max_u32.bin", 16 + 16),
        ("03_data_truncated.bin", 16 + 256),
        ("04_step_zero.bin", 16 + 16),
        ("05_width_zero.bin", 16 + 16),
        ("06_junk_encoding.bin", 16 + 12),
    ]
}
