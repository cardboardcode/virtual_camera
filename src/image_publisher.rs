use opencv::core::{Mat};
use opencv::prelude::MatTraitConst;
use opencv::{
    imgcodecs, Result,
};
use opencv::{
    core,
    prelude::*,
    videoio,
};

use std::{sync::Arc, thread, time::Duration};
use builtin_interfaces::msg::Time as BuiltinTime;
use rclrs::*;

struct ImagePublisher {
    node: Arc<Node>,
    publisher: Arc<Publisher<sensor_msgs::msg::Image>>,
    image: Result<Mat>,
}

impl ImagePublisher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("image_publisher").unwrap();
        let publisher = node.create_publisher("/virtual_camera/image_raw").unwrap();

        let image_path = "/workspace/data/test.jpg";

        // TODO(cardboardcode): Implement feature to automatically detect if video or image

        // Read the image using imread
        let test_image = imgcodecs::imread(image_path, imgcodecs::IMREAD_COLOR);

        Ok(Self { 
            node: node,
            publisher : publisher,
            image: test_image,
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
        
        let clock = self.node.get_clock();
        let now: Time = clock.now();
        let duration = std::time::Duration::from_nanos(now.nsec.try_into().unwrap());

        let image_msg = self.mat_to_ros_image_dynamic_encoding(input_img);
        // let image_msg = self.image.as_ref().map(|img| self.mat_to_ros_image_dynamic_encoding(img));

        let mut local_image_msg = image_msg.unwrap();

        let real_image_msg = sensor_msgs::msg::Image {
            header: std_msgs::msg::Header {
                stamp: BuiltinTime{
                     sec: duration.as_secs().try_into().unwrap(),
                     nanosec: duration.subsec_nanos().try_into().unwrap(),
                },
                frame_id: "map".to_string(),
            },
            height: local_image_msg.height,
            width: local_image_msg.width,
            encoding: local_image_msg.encoding.clone(),
            is_bigendian: 0,
            step: local_image_msg.step,
            data: local_image_msg.data.clone(),
        };
        self.publisher.publish(real_image_msg)?;
        Ok(increment + 1)
    }
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
    if cam.is_opened().unwrap() {
        println!("{} is a video file.", file_name);
    }
    let mut frame = core::Mat::default();
    let frame_read = videoio::VideoCapture::read(&mut cam, &mut frame); 

    // Try to open the file as an image.
    let image = imgcodecs::imread(file_name, imgcodecs::IMREAD_COLOR).unwrap();
    if !image.empty() {
        println!("{} is an image file.", file_name);
    }

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(42));
        let success = videoio::VideoCapture::read(&mut cam, &mut frame);

        if !success.unwrap() {
            println!("Restarting video from the beginning.");
            cam.set(1, 0.0); // 1 is the property ID for frame position
            continue; // Go back to the beginning of the loop
        }

        count = publisher_other_thread.publish_data(count, &mut frame).unwrap();
        println!("Publishing [test image] - {}", cursor[count]);

        if count == 3 {
            count = 0;
        }

    });
    executor.spin(SpinOptions::default()).first_error()
}
