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

// Fuzz target for the `sensor_msgs::Image` -> OpenCV `Mat` decode path
// (`ImageViewer::decode_image`).
//
// Input layout (4 + 4 + 4 + 4 = 16 bytes of little-endian u32 prefix, followed
// by the raw pixel payload):
//
//     offset  0 : u32 LE  width
//     offset  4 : u32 LE  height
//     offset  8 : u32 LE  step
//     offset 12 : u32 LE  encoding_index (mod ENCODINGS.len())
//     offset 16: remaining bytes become `msg.data`
//
// The target performs no assertions; crashes only surface as real memory-safety
// violations in the decode path.

#![no_main]
use libfuzzer_sys::fuzz_target;
use virtual_camera::{Header, SensorImage, Time, viewer::ImageViewer};

const ENCODINGS: [&str; 8] = [
    "mono8",
    "bgr8",
    "rgb8",
    "mono16",
    "rgba8",
    "bgra8",
    "32FC1",
    "junk_encoding",
];

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }

    let width = u32::from_le_bytes(data[0..4].try_into().unwrap());
    let height = u32::from_le_bytes(data[4..8].try_into().unwrap());
    let step = u32::from_le_bytes(data[8..12].try_into().unwrap());
    let encoding_index = u32::from_le_bytes(data[12..16].try_into().unwrap());
    let payload = &data[16..];

    let encoding = ENCODINGS[encoding_index as usize % ENCODINGS.len()].to_string();

    let msg = SensorImage {
        header: Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: "world".to_string(),
        },
        height,
        width,
        encoding,
        is_bigendian: 0,
        step,
        data: payload.to_vec(),
    };

    let _ = ImageViewer::decode_image(&msg);
});
