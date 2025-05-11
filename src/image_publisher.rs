use std::{fs,io,path::Path};
use std::io::Write;
use std::{sync::Arc, thread, time::Duration};
use opencv::core::Mat;
use opencv::{
    core,
    imgcodecs,
    prelude::*,
    Result,
    videoio,
};
use builtin_interfaces::msg::Time as BuiltinTime;
use rclrs::*;

struct ImagePublisher {
    node: Arc<Node>,
    publisher: Arc<Publisher<sensor_msgs::msg::Image>>,
}

impl ImagePublisher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("image_publisher").unwrap();
        let publisher = node.create_publisher("/virtual_camera/image_raw").unwrap();

        Ok(Self { 
            node: node,
            publisher : publisher,
        })
    }

    /// Manually convert an OpenCV Mat to a ROS 2 Image message with dynamic encoding
    fn mat_to_ros_image_dynamic_encoding(&self, mat: &Mat) -> Result<sensor_msgs::msg::Image> {
        let size = mat.size()?;
        let height = size.height as u32;
        let width = size.width as u32;
        let channels = mat.channels();
        let step = channels as u32 * width;
        let depth = mat.depth();
        
        let data_ptr = mat.data();

        // Calculate the total number of bytes in the Mat
        let total_bytes = mat.total() * mat.elem_size().unwrap();

        // Create a Vec<u8> with the capacity to hold the data
        let mut data_vec: Vec<u8> = Vec::with_capacity(total_bytes as usize);

        unsafe {
            // Create a slice from the raw pointer and length
            let data_slice = std::slice::from_raw_parts(data_ptr, total_bytes as usize);

            // Extend the Vec<u8> with the data from the slice
            data_vec.extend_from_slice(data_slice);
        }
        
        let data = data_vec;

        // TODO(cardboardcode): Implement proper error handling using opencv::Error

        let encoding = match (depth, channels) {
            (opencv::core::CV_8U, 1) => "mono8",
            (opencv::core::CV_8U, 3) => "bgr8", // Assuming default imread is BGR
            (opencv::core::CV_8U, 4) => "bgra8", // Assuming if it has 4 channels, it's BGRA
            (opencv::core::CV_16U, 1) => "mono16",
            (i32::MIN..=-1_i32, _) | (1_i32, _) | (3_i32..=i32::MAX, _) => todo!(),
            _ => todo!(),
        };

        let clock = self.node.get_clock();
        let now: Time = clock.now();
        let duration = std::time::Duration::from_nanos(now.nsec.try_into().unwrap());

        let msg = sensor_msgs::msg::Image {
            header: std_msgs::msg::Header {
                stamp: BuiltinTime{
                     sec: duration.as_secs().try_into().unwrap(),
                     nanosec: duration.subsec_nanos().try_into().unwrap(),
                },
                frame_id: "map".to_string(),
            },
            height,
            width,
            encoding: encoding.to_string(),
            is_bigendian: 0, // Assuming little-endian
            step,
            data,
        };

        Ok(msg)
    }

    fn publish_data(&self, increment: usize, input_img: &mut Mat) -> Result<usize, RclrsError> {  

        let image_msg = self.mat_to_ros_image_dynamic_encoding(input_img).unwrap();

        self.publisher.publish(image_msg)?;
        Ok(increment + 1)
    }
}

fn detect_file_type(file_path: &Path) -> io::Result<String> {
    let metadata = fs::metadata(file_path)?;
    if !metadata.is_file() {
        return Ok("Not a regular file".to_string());
    }

    // Read the first few bytes of the file to try and identify the magic number.
    let mut file = fs::File::open(file_path)?;
    let mut buffer = [0; 16]; // Read up to 16 bytes
    let bytes_read = io::Read::read(&mut file, &mut buffer)?;

    if bytes_read == 0 {
        return Ok("Empty file".to_string());
    }

    println!("buffer = {:?}", buffer);

    // Simple magic number detection (not exhaustive)
    if buffer.starts_with(&[0xFF, 0xD8, 0xFF]) { // JPEG
        return Ok("Image".to_string());
    } else if buffer.starts_with(&[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]) { //PNG
        return Ok("Image".to_string());
    } else if buffer.starts_with(&[0x47, 0x49, 0x46, 0x38]) { // GIF
        return Ok("Image".to_string());
    } else if buffer.starts_with(&[0x66, 0x74, 0x79, 0x70, 0x6D, 0x70, 0x34, 0x32]) // ftypmp42, ftypisom, etc.
        || buffer.starts_with(&[0x00, 0x00, 0x01, 0xBA]) // MPEG program stream
        || buffer.starts_with(&[0x00, 0x00, 0x01, 0xB3]) // MPEG video stream
        || buffer.starts_with(&[0x4F, 0x67, 0x67, 0x53]) { // Ogg format (WebM, Ogg Vorbis, etc.)
        return Ok("Video".to_string());
    } else if buffer.starts_with(&[0x00, 0x00, 0x00, 0x18]) { // More MP4 variants
        return Ok("Video".to_string());
    } else if buffer.starts_with(&[0x1A, 0x45, 0xDF, 0xA3, 0xA3, 0x42]) { // MKV variants
        return Ok("Video".to_string());
    } else if buffer.starts_with(&[0x1A, 0x45, 0xDF]) { // WEBM variants
        return Ok("Video".to_string());
    }

    Ok("a file type that cannot be determined.".to_string())
}
fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let publisher = Arc::new(ImagePublisher::new(&executor).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: usize = 0;
    let cursor = ['/', '-', '\\', '|'];

    let file_name = "/workspace/data/input_data";

    // Check if the file exists.
    if !std::path::Path::new(file_name).exists() {
        println!("File does not exist: {}", file_name);
        return Ok(()); // Exit gracefully
    }

    // Try to open the file as a video.
    let mut cam = videoio::VideoCapture::from_file(&file_name, videoio::CAP_ANY).unwrap();

    let mut frame = core::Mat::default();
    let frame_read = videoio::VideoCapture::read(&mut cam, &mut frame);
    let test_image = imgcodecs::imread(file_name, imgcodecs::IMREAD_COLOR).unwrap();

    let file_path = Path::new(file_name);
    let mut file_type = detect_file_type(file_path).unwrap();

    let mut is_file_a_video: bool = false;
    let mut is_file_an_image: bool = false;

    if file_type.clone() == "Video" {
        println!("file is a video.");
        is_file_a_video = true;
    } else if file_type.clone() == "Image" {
        println!("file is a image.");
        is_file_an_image = true;
    }

    let mut success: bool = false;

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(42));

        if is_file_a_video {
            success = videoio::VideoCapture::read(&mut cam, &mut frame).unwrap();
        }
        if is_file_an_image {
            success = true;
            frame = test_image.clone();
        }

        if !success {
            println!("Restarting video from the beginning.");
            cam.set(1, 0.0); // 1 is the property ID for frame position
            continue; // Go back to the beginning of the loop
        }

        count = publisher_other_thread.publish_data(count, &mut frame).unwrap();
        println!("\rPublishing [test image] - {}", cursor[count]);
        io::stdout().flush().unwrap();

        if count == 3 {
            count = 0;
        }

    });
    executor.spin(SpinOptions::default()).first_error()
}
