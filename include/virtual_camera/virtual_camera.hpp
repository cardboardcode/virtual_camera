// Copyright 2021 Bey Hao Yun
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

#ifndef VIRTUAL_CAMERA__VIRTUAL_CAMERA_HPP_
#define VIRTUAL_CAMERA__VIRTUAL_CAMERA_HPP_

#include <chrono>
#include <string>
#include <functional>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

// OpenCV library
#include "opencv2/opencv.hpp"

#define FILE_PATH_TO_VIDEO "/data/input_data"
#define FILE_PATH_TO_DEFAULT_IMAGE "/data/nadezhda_diskant.jpg"
// #define FPS 24

/*! \class VirtualCamera
    \brief The sole class object in this package.
    The VirtualCamera class object utilizes opencv and
    outputs images as sensor_msgs Image ROS message in ROS2.
*/
class VirtualCamera : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  /*!< \brief A variable to set the interval in which each frame is published */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  /*!< \brief A ROS2 subscriber service */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  /*!< \brief A ROS2 publisher service */
  cv::VideoCapture cap;
  /*!< \brief A VideoCapture object to read a video file as defined in
   * FILE_PATH_TO_VIDEO
   */
  bool useImage;
  /*!< \brief A boolean that enables flexibility between reading video or image */
  int pos = 0;
  /*!< \brief A counter to keep track of the position of the output cursor */
  char cursor[4] = {'/', '-', '\\', '|'};
  /*!< \brief An array of char which show cursors at different stages. */
  std::string FILE_PATH_TO_PACKAGE;
  int fps = 24;
  cv::Mat img;

public:
  mutable bool shuttingDown;
  /*! \constructor Constructor
    *
    *  1. Set bool useImage to false.
    *  2. Create publisher, publishing to topic "/virtual_camera/image_raw"
    *  3. Create subscriber, subscribing to topic "virtual_camera/state_input"
    *  4. Attempt open of video file. Check if open was successfully. Otherwise,
    *  set useImage to true.
    *
    */
  VirtualCamera()
  : Node("virtual_camera")
  {
    useImage = false;
    shuttingDown = false;
    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/virtual_camera/image_raw", 10);
    // Set publisher to publish message at set time interval.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / fps), std::bind(&VirtualCamera::timer_callback, this));

    std::string temp(PATH_TO_PACKAGE);

    FILE_PATH_TO_PACKAGE = temp;

    printf("%s\n", (FILE_PATH_TO_PACKAGE + FILE_PATH_TO_VIDEO).c_str());

    cap.open(FILE_PATH_TO_PACKAGE + FILE_PATH_TO_VIDEO);

    // Check if camera opened successfully
    if (!cap.isOpened()) {
      std::cout << "Error opening video file. Opening as image instead." << std::endl;
      useImage = true;
    }

    // If reading image, check if image symbolic link file can be accessed properly
    if (useImage) {
      img = cv::imread(FILE_PATH_TO_PACKAGE + FILE_PATH_TO_VIDEO, cv::IMREAD_COLOR);
      // If failed to load image proper, load default image.
      if (img.empty()) {
        img = cv::imread(FILE_PATH_TO_PACKAGE + FILE_PATH_TO_DEFAULT_IMAGE, cv::IMREAD_COLOR);
      }
    }

    this->declare_parameter("FPS");
    this->set_parameter(rclcpp::Parameter("FPS", 24));
  }

  void activate_time_callback(void)
  {
    this->timer_callback();
  }

  void activate_advance_cursor(void)
  {
    this->advance_cursor();
  }

  sensor_msgs::msg::Image process_timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing Image");
    rclcpp::Parameter int_param = this->get_parameter("FPS");
    int new_fps = int_param.as_int();

    if (fps != new_fps) {
      // ros2 param set /virtual_camera FPS 100
      RCLCPP_WARN(this->get_logger(), "Changing FPS to %d.", new_fps);
      fps = new_fps;
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / fps), std::bind(&VirtualCamera::timer_callback, this));
    }

    cv::Mat frame;

    if (!useImage) {
      // Capture frame-by-frame
      cap >> frame;

      // If the frame is empty, break immediately
      if (frame.empty()) {
        cap.open(FILE_PATH_TO_PACKAGE + FILE_PATH_TO_VIDEO);
        cap >> frame;
      }

    } else {
      frame = img;
    }

    if (frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Discarding faulty image. [input_data] symlink broken.");
      throw std::runtime_error("Please check the symbolic link file you are using as input_data.");
    }
    // Implement progress wheel here.
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    msg->header.frame_id = "world";

    return *msg;
  }

private:
  /*! \brief
   * Acts as an active publisher of images taken by VideoCapture and outputs
   * as ROS2 sensor_msgs Image messages.\n
   * Checks if VideoCapture reads valid video file.\n
   * If true, convert and publish cv::Mat object to ROS2 message.
   * Otherwise, open as Image and publish.\n
   * Conducts check if final Image or Video file is empty or not.\n
   * If empty, discard output and generate warning.
   * Otherwise, publish.
   */
  void timer_callback()
  {
    sensor_msgs::msg::Image msg;

    msg = this->process_timer_callback();

    publisher_->publish(msg);
    advance_cursor();
  }

  /*! \brief
   * Cycles through all visual stages of cursor
   * to show publication is in progress.
   */
  void advance_cursor()
  {
    printf("\rPublishing %c\b", cursor[pos]);
    fflush(stdout);
    pos = (pos + 1) % 4;
  }
};

#endif  // VIRTUAL_CAMERA__VIRTUAL_CAMERA_HPP_
