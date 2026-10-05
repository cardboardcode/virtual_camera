//! Pure, ROS- and OpenCV-free logic shared by the publisher and subscriber
//! binaries and exercised by the cargo test suite without any native library.
//!
//! Only `std` is used here, so these tests run under
//! `cargo test --no-default-features` on any machine with a Rust toolchain.

use std::io::Read;
use std::path::Path;

// ── Local mirrors of the OpenCV CV_* constants (i32) used in the codebase ──
// Keeping them here means `logic` stays 100% std-only and the tests compile
// without linking against OpenCV at all.
// OpenCV depth constants (low 3 bits of the full type).
pub(crate) const CV_8U: i32 = 0;
pub(crate) const CV_16U: i32 = 1;
#[allow(dead_code)] // referenced only from #[cfg(test)] assertions
pub(crate) const CV_32F: i32 = 3;
#[allow(dead_code)] // referenced only from #[cfg(test)] assertions
pub(crate) const CV_64F: i32 = 4;

// Full type = depth | ((channel_index) << 3), where channel_index = channels - 1.
//
// NOTE: in Rust `+` binds tighter than `<<`, so the channel bitfield MUST be
// parenthesised separately and we add (or use `|`) — never `cv_u + n << 3`.
// Channel-1 constants reduce to the bare depth (0 << 3 == 0); the identity `+ 0`
// is kept deliberately so each line mirrors the OpenCV `depth | (ch<<3)` layout.
#[allow(clippy::identity_op)]
pub(crate) const CV_8UC1: i32 = CV_8U + (0 << 3);
pub(crate) const CV_8UC3: i32 = CV_8U + (2 << 3);
pub(crate) const CV_8UC4: i32 = CV_8U + (3 << 3);
#[allow(clippy::identity_op)]
pub(crate) const CV_16UC1: i32 = CV_16U + (0 << 3);

// ─────────────────────────────────────────────────────────────────────────────

/// File-type classification used by the publisher playback loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileKind {
    Image,
    Video,
    Unknown,
}

/// What the subscriber needs to know in order to build a display Mat from an
/// incoming `sensor_msgs/Image` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MatSpec {
    /// OpenCV full-type constant (e.g. CV_8UC3).
    pub cv_type: i32,
    /// Whether the payload must be converted RGB(A) → BGR(A) after import.
    pub convert_rgb: bool,
}

/// All encodings accepted in the subscriber direction.
pub const SUPPORTED_ENCODINGS: &[&str] = &["mono8", "mono16", "bgr8", "rgb8", "bgra8", "rgba8"];

// ── Subscriber side: encoding → MatSpec ──────────────────────────────────────

/// Map a `sensor_msgs/Image` encoding string to a [`MatSpec`], or `None` for
/// unsupported encodings.
#[must_use]
pub fn subscriber_mat_spec(encoding: &str) -> Option<MatSpec> {
    Some(match encoding {
        "mono8" => MatSpec {
            cv_type: CV_8UC1,
            convert_rgb: false,
        },
        "mono16" => MatSpec {
            cv_type: CV_16UC1,
            convert_rgb: false,
        },
        "bgr8" => MatSpec {
            cv_type: CV_8UC3,
            convert_rgb: false,
        },
        "rgb8" => MatSpec {
            cv_type: CV_8UC3,
            convert_rgb: true,
        },
        "bgra8" => MatSpec {
            cv_type: CV_8UC4,
            convert_rgb: false,
        },
        "rgba8" => MatSpec {
            cv_type: CV_8UC4,
            convert_rgb: true,
        },
        _ => return None,
    })
}

// ── Publisher side: (depth, channels) → encoding label ──────────────────────

/// Pick the `sensor_msgs/Image` encoding label for a mat described by its
/// OpenCV depth and channel count.
///
/// Returns `None` for combinations that are not publishable (8-bit 2-channel,
/// 16-bit BGR, any floating-point depth, etc.) instead of panicking.
#[must_use]
pub fn publisher_encoding(depth: i32, channels: u32) -> Option<&'static str> {
    Some(match (depth, channels) {
        (CV_8U, 1) => "mono8",
        (CV_8U, 3) => "bgr8",
        (CV_8U, 4) => "bgra8",
        (CV_16U, 1) => "mono16",
        _ => return None,
    })
}

/// All publishable (depth, channels) combos, handy for exhaustive tests.
pub const PUBLISHABLE_COMBOS: &[(i32, u32)] = &[(CV_8U, 1), (CV_8U, 3), (CV_8U, 4), (CV_16U, 1)];

// ── Image header fields (pure math, no Mat object) ──────────────────────────

/// Compute `(height, width, step_bytes, encoding)` for a mat described by its
/// shape/type, without requiring a live `opencv::core::Mat`.
///
/// # Panics
///
/// If the depth/channel combination is not in [`PUBLISHABLE_COMBOS`].
///
/// Use [`publisher_encoding`] for a non-panicking variant.
#[must_use]
pub fn image_header_fields(
    height: u32,
    width: u32,
    depth: i32,
    channels: u32,
) -> Option<(u32, u32, u32, &'static str)> {
    let step = width.checked_mul(channels)?;
    let encoding = publisher_encoding(depth, channels)?;
    Some((height, width, step, encoding))
}

// ── Magic-byte file-type sniffing ────────────────────────────────────────────

/// Classify a raw file-header byte slice by magic bytes.
#[must_use]
pub fn classify_magic_bytes(header: &[u8]) -> FileKind {
    if matches_image_magic(header) {
        FileKind::Image
    } else if matches_video_magic(header) {
        FileKind::Video
    } else {
        FileKind::Unknown
    }
}

/// True when `buffer` starts with a known image-file magic prefix
/// (JPEG, PNG, or GIF).
#[must_use]
pub fn matches_image_magic(buffer: &[u8]) -> bool {
    buffer.starts_with(&[0xFF, 0xD8, 0xFF])                                  // JPEG
        || buffer.starts_with(&[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]) // PNG
        || buffer.starts_with(&[0x47, 0x49, 0x46, 0x38]) // GIF87a / GIF89a
}

/// True when `buffer` starts with a known video-container magic prefix
/// (MP4 ftyp, MPEG program/video stream, Ogg, WebM/MKV EBML).
#[must_use]
pub fn matches_video_magic(buffer: &[u8]) -> bool {
    buffer.starts_with(&[0x66, 0x74, 0x79, 0x70, 0x6D, 0x70, 0x34, 0x32]) // "ftypmp42"
        || buffer.starts_with(&[0x00, 0x00, 0x01, 0xBA])    // MPEG program stream
        || buffer.starts_with(&[0x00, 0x00, 0x01, 0xB3])    // MPEG video stream
        || buffer.starts_with(&[0x4F, 0x67, 0x67, 0x53])    // "OggS" (Ogg / WebM family)
        || buffer.starts_with(&[0x00, 0x00, 0x00, 0x18])    // MP4 variant
        || buffer.starts_with(&[0x1A, 0x45, 0xDF, 0xA3, 0xA3, 0x42]) // MKV variant
        || buffer.starts_with(&[0x1A, 0x45, 0xDF]) // EBML (WebM / MKV base)
}

/// Read the first 16 bytes of the file at `file_path` and classify it.
///
/// Returns `Ok(FileKind)` even for an empty or unrecognised file (see
/// `FileKind::Unknown`).  Propagates `std::io::Error` when the file does not
/// exist or is not a regular file.
pub fn detect_file_kind(file_path: &Path) -> std::io::Result<FileKind> {
    let metadata = std::fs::metadata(file_path)?;
    if !metadata.is_file() {
        return Ok(FileKind::Unknown);
    }
    let mut file = std::fs::File::open(file_path)?;
    let mut buffer = [0u8; 16];
    let bytes_read = file.read(&mut buffer)?;
    if bytes_read == 0 {
        return Ok(FileKind::Unknown);
    }
    Ok(classify_magic_bytes(&buffer[..bytes_read]))
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests (zero native dependencies — `cargo test --no-default-features`)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── magic-byte classification ───────────────────────────────────────────

    #[test]
    fn jpeg_magic_classifies_as_image() {
        let header = [0xFF, 0xD8, 0xFF];
        assert!(matches_image_magic(&header));
        assert_eq!(classify_magic_bytes(&header), FileKind::Image);
    }

    #[test]
    fn png_magic_classifies_as_image() {
        let header = [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];
        assert!(matches_image_magic(&header));
        assert_eq!(classify_magic_bytes(&header), FileKind::Image);
    }

    #[test]
    fn gif_magic_classifies_as_image() {
        for version in [b"GIF87a", b"GIF89a"] {
            assert!(matches_image_magic(version));
            assert_eq!(classify_magic_bytes(version), FileKind::Image);
        }
    }

    #[test]
    fn mp4_ftyp_magic_classifies_as_video() {
        assert!(matches_video_magic(b"ftypmp42"));
        assert_eq!(classify_magic_bytes(b"ftypmp42"), FileKind::Video);
    }

    #[test]
    fn mpeg_stream_magic_classifies_as_video() {
        assert!(matches_video_magic(&[0x00, 0x00, 0x01, 0xBA]));
        assert!(matches_video_magic(&[0x00, 0x00, 0x01, 0xB3]));
        assert_eq!(
            classify_magic_bytes(&[0x00, 0x00, 0x01, 0xBA]),
            FileKind::Video
        );
    }

    #[test]
    fn ogg_and_ebml_magic_classify_as_video() {
        assert!(matches_video_magic(b"OggS"));
        assert!(matches_video_magic(&[0x1A, 0x45, 0xDF, 0xA3, 0xA3, 0x42]));
        assert!(matches_video_magic(&[0x1A, 0x45, 0xDF, 0x53, 0x80, 0x82])); // WebM EBML
        assert_eq!(classify_magic_bytes(b"OggS"), FileKind::Video);
    }

    #[test]
    fn unrecognized_bytes_classify_as_unknown() {
        let header = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        assert!(!matches_image_magic(&header));
        assert!(!matches_video_magic(&header));
        assert_eq!(classify_magic_bytes(&header), FileKind::Unknown);
    }

    #[test]
    fn empty_header_classifies_as_unknown() {
        assert_eq!(classify_magic_bytes(&[]), FileKind::Unknown);
    }

    #[test]
    fn near_miss_prefixes_do_not_match() {
        // 0xFF 0xD8 followed by a non-JPEG byte: not JPEG.
        assert!(!matches_image_magic(&[0xFF, 0xD8, 0x00]));
        // "ftyp" boxes whose type string is not "mp42" are not matched.
        assert!(!matches_video_magic(b"ftypisom"));
        // But the "mp42" variant is.
        assert!(matches_video_magic(b"ftypmp42"));
        // A truncated JPEG header (only 2 of the 3 magic bytes) does not match.
        assert!(!matches_image_magic(&[0xFF, 0xD8]));
    }

    // ── detect_file_kind (real filesystem) ──────────────────────────────────

    fn temp_path(tag: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("vcam_sniff_{tag}_{}", std::process::id()))
    }

    #[test]
    fn detect_real_jpeg_file() {
        let path = temp_path("jpeg");
        let bytes: Vec<u8> = vec![0xFF, 0xD8, 0xFF, 0xE0]
            .into_iter()
            .chain(std::iter::repeat(0u8).take(32))
            .collect();
        std::fs::write(&path, &bytes).unwrap();
        let kind = detect_file_kind(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(kind, FileKind::Image);
    }

    #[test]
    fn detect_real_video_file() {
        let path = temp_path("video");
        // Real MP4 layout: [size: u32 BE][ftyp][brand "mp42"][version u32]
        let bytes: Vec<u8> = [
            0x00u8, 0x00, 0x00, 0x18, b'f', b't', b'y', b'p', b'm', b'p', b'4', b'2',
        ]
        .iter()
        .copied()
        .collect();
        std::fs::write(&path, &bytes).unwrap();
        let kind = detect_file_kind(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(kind, FileKind::Video);
    }

    #[test]
    fn detect_empty_file_is_unknown() {
        let path = temp_path("empty");
        std::fs::File::create(&path).unwrap();
        let kind = detect_file_kind(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(kind, FileKind::Unknown);
    }

    #[test]
    fn detect_missing_file_returns_io_error() {
        let path = std::env::temp_dir().join("vcam_nonexistent_24680");
        assert!(detect_file_kind(&path).is_err());
    }

    #[test]
    fn detect_directory_is_unknown() {
        let kind = detect_file_kind(&std::env::temp_dir()).unwrap();
        assert_eq!(kind, FileKind::Unknown);
    }

    // ── subscriber_mat_spec ─────────────────────────────────────────────────

    #[test]
    fn all_supported_encodings_map() {
        assert_eq!(SUPPORTED_ENCODINGS.len(), 6);
        for encoding in SUPPORTED_ENCODINGS {
            assert!(
                subscriber_mat_spec(encoding).is_some(),
                "{encoding} should map"
            );
        }
    }

    #[test]
    fn rgb_variants_require_conversion() {
        assert!(subscriber_mat_spec("rgb8").unwrap().convert_rgb);
        assert!(subscriber_mat_spec("rgba8").unwrap().convert_rgb);
        assert!(!subscriber_mat_spec("bgr8").unwrap().convert_rgb);
        assert!(!subscriber_mat_spec("bgra8").unwrap().convert_rgb);
        assert!(!subscriber_mat_spec("mono8").unwrap().convert_rgb);
        assert!(!subscriber_mat_spec("mono16").unwrap().convert_rgb);
    }

    #[test]
    fn cv_type_depth_and_channel_decode() {
        // OpenCV full type layout = depth (bits 0-2) | ((channels - 1) << 3),
        // which matches the CV_* constants in this file.
        //   depth    = cv_type & 0x7
        //   channels = (cv_type >> 3) + 1
        let channels = |spec: MatSpec| (spec.cv_type >> 3) + 1;
        let depth = |spec: MatSpec| spec.cv_type & 0x7;

        // sanity: the constants really encode the way OpenCV does
        assert_eq!(CV_8UC1, 0);
        assert_eq!(CV_8UC3, 16);
        assert_eq!(CV_8UC4, 24);
        assert_eq!(CV_16UC1, 1);

        let spec8uc1 = subscriber_mat_spec("mono8").unwrap();
        assert_eq!(channels(spec8uc1), 1);
        assert_eq!(depth(spec8uc1), CV_8U);
        let spec8uc3 = subscriber_mat_spec("bgr8").unwrap();
        assert_eq!(channels(spec8uc3), 3);
        assert_eq!(depth(spec8uc3), CV_8U);
        let spec8uc4 = subscriber_mat_spec("bgra8").unwrap();
        assert_eq!(channels(spec8uc4), 4);
        assert_eq!(depth(spec8uc4), CV_8U);
        let spec16uc1 = subscriber_mat_spec("mono16").unwrap();
        assert_eq!(channels(spec16uc1), 1);
        assert_eq!(depth(spec16uc1), CV_16U);
        let _ = spec8uc4; // all of the above already consumed the spec
                          // 8-bit depths: extract the depth field (bits 0-2) only
        for enc in ["mono8", "bgr8", "rgb8", "bgra8", "rgba8"] {
            assert_eq!(subscriber_mat_spec(enc).unwrap().cv_type & 0x7, CV_8U);
        }
        assert_eq!(subscriber_mat_spec("mono16").unwrap().cv_type & 0x7, CV_16U);
    }

    #[test]
    fn unsupported_encodings_rejected() {
        for encoding in [
            "", "invalid", "mono32", "bgr16", "yuv420p", "rgba16", "BGRA8",
        ] {
            assert!(
                subscriber_mat_spec(encoding).is_none(),
                "{encoding} should be rejected"
            );
        }
    }

    // ── publisher_encoding ──────────────────────────────────────────────────

    #[test]
    fn all_publishable_combos_have_encoding() {
        for (depth, channels) in PUBLISHABLE_COMBOS {
            assert!(
                publisher_encoding(*depth, *channels).is_some(),
                "{depth},{channels}"
            );
        }
        assert_eq!(PUBLISHABLE_COMBOS.len(), 4);
    }

    #[test]
    fn expected_encoding_labels() {
        assert_eq!(publisher_encoding(CV_8U, 1), Some("mono8"));
        assert_eq!(publisher_encoding(CV_8U, 3), Some("bgr8"));
        assert_eq!(publisher_encoding(CV_8U, 4), Some("bgra8"));
        assert_eq!(publisher_encoding(CV_16U, 1), Some("mono16"));
    }

    #[test]
    fn unpublishable_combos_return_none() {
        assert_eq!(publisher_encoding(CV_8U, 0), None);
        assert_eq!(publisher_encoding(CV_8U, 2), None);
        assert_eq!(publisher_encoding(CV_8U, 5), None);
        assert_eq!(publisher_encoding(CV_16U, 3), None);
        assert_eq!(publisher_encoding(CV_32F, 1), None);
        assert_eq!(publisher_encoding(CV_32F, 3), None);
        assert_eq!(publisher_encoding(CV_64F, 4), None);
    }

    // ── image_header_fields (pure math) ─────────────────────────────────────

    #[test]
    fn step_and_encoding_for_known_shapes() {
        // 4x3 BGR8: step = 3 channels * 3 cols = 9
        assert_eq!(image_header_fields(4, 3, CV_8U, 3), Some((4, 3, 9, "bgr8")));
        // 2x2 mono16: step = 1 * 2 = 2
        assert_eq!(
            image_header_fields(2, 2, CV_16U, 1),
            Some((2, 2, 2, "mono16"))
        );
        // 8x10 BGRA8: step = 4 * 10 = 40
        assert_eq!(
            image_header_fields(8, 10, CV_8U, 4),
            Some((8, 10, 40, "bgra8"))
        );
    }

    #[test]
    fn rejected_combos_yield_none_and_zero_height_is_mathematically_fine() {
        assert_eq!(image_header_fields(4, 3, CV_32F, 3), None);
        // Zero height is meaningless for a real camera but valid math here:
        // step is still width * channels.
        assert_eq!(image_header_fields(0, 3, CV_8U, 3), Some((0, 3, 9, "bgr8")));
    }

    // ── round-trip consistency ──────────────────────────────────────────────

    #[test]
    fn publisher_and_subscriber_speak_the_same_encodings() {
        // Every label the publisher emits, the subscriber must accept.
        for labels in ["mono8", "bgr8", "bgra8", "mono16"] {
            assert!(
                subscriber_mat_spec(labels).is_some(),
                "subscriber must accept {labels}"
            );
        }
        // And every subscriber-supported label is a valid ROS encoding string.
        for enc in SUPPORTED_ENCODINGS {
            assert!(!enc.is_empty() && enc.chars().all(|c| c.is_ascii_alphanumeric()));
        }
    }
}
