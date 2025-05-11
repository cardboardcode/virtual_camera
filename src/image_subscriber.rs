use opencv::core::{Mat, Size, Scalar};
use opencv::highgui;
use opencv::{
    core::{CV_8UC1, CV_8UC3, CV_8UC4, CV_16UC1},
    prelude::*,
    imgproc,
};

use anyhow::{Error, Result};
use rclrs::*;

/// Manually convert a ROS 2 Image message to OpenCV Mat
fn ros_image_to_mat(msg: &sensor_msgs::msg::Image) -> Result<Mat, opencv::Error> {
    // Determine OpenCV type based on encoding
    let (cv_type, convert_rgb): (i32, bool) = match msg.encoding.as_str() {
        "mono8" => (CV_8UC1, false),
        "mono16" => (CV_16UC1, false),
        "bgr8" => (CV_8UC3, false),
        "rgb8" => (CV_8UC3, true),
        "bgra8" => (CV_8UC4, false),
        "rgba8" => (CV_8UC4, true),
        other => {
            return Err(opencv::Error::new(0, format!("Unsupported encoding: {}", other)));
        }
    };

    println!(
        "Encoding: {}, Size: {}x{}, Step: {}, Data len: {}",
        msg.encoding,
        msg.width,
        msg.height,
        msg.step,
        msg.data.len()
    );

    // Create Mat from raw buffer
    // let mat = Mat::new_rows_cols_with_data(
    //     msg.height as i32,
    //     msg.width as i32,
    //     &msg.data
    // )?;

    let mat = unsafe {
        Mat::new_rows_cols_with_data_unsafe(
        msg.height as i32,
        msg.width as i32,
        cv_type,
        msg.data.as_ptr() as *mut std::ffi::c_void,
        msg.step as usize,
        )?
    };

    // Convert if necessary (e.g., RGB → BGR)
    if convert_rgb {
        let mut converted = Mat::default();
        if msg.encoding == "rgb8" {
            imgproc::cvt_color(&mat, &mut converted, imgproc::COLOR_RGB2BGR, 0)?;
        } else if msg.encoding == "rgba8" {
            imgproc::cvt_color(&mat, &mut converted, imgproc::COLOR_RGBA2BGRA, 0)?;
        }
        Ok(converted)
    } else {
        Ok(mat.try_clone()?) // Return cloned Mat to own memory safely
    }
}

fn image_msg_to_mat(msg: &sensor_msgs::msg::Image) -> opencv::Result<Mat> {

    println!(
        "Received image with width: {}, height: {}, encoding: {}",
        msg.width,
        msg.height,
        msg.encoding
    );

    let size = Size::new(msg.width.try_into().unwrap(), msg.height.try_into().unwrap());
    let white_color = Scalar::new(255.0, 255.0, 255.0, 0.0); // BGR (and alpha, though not used here for a 3-channel image)
    let image = Mat::new_size_with_default(size, CV_8UC3, white_color)?;
    Ok(image)
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("image_subscriber")?;

    let mut num_messages: usize = 0;

    let _subscription = node.create_subscription::<sensor_msgs::msg::Image, _>(
        "/virtual_camera/image_raw",
        move |msg: sensor_msgs::msg::Image| {
            num_messages += 1;
            println!("Image received...");
            // let white_image = image_msg_to_mat(&msg);

            match ros_image_to_mat(&msg){
                Ok(mat) => {
                    // Successfully got the Mat object
                    // println!("Image dimensions: {}x{}", mat.rows, mat.cols);
                    // Perform operations on 'mat'
                    highgui::imshow("Image Subscriber", &mat);
                    highgui::wait_key(1);
                }
                Err(err) => {
                    // An error occurred
                    eprintln!("Error processing image: {}", err);
                    // Handle the error appropriately (e.g., log, return a default, exit)
                }
            }


            println!("(Got {} messages so far)", num_messages);
        },
    )?;

    executor
        .spin(SpinOptions::default())
        .first_error()
        .map_err(|err| err.into())
}
