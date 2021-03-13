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

#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include <chrono>
#include <string>
#include <memory>
#include <functional>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

// OpenCV library
#include "opencv2/opencv.hpp"
#include "virtual_camera/image_viewer.hpp"

using namespace std::chrono_literals;

std::string PATH_TO_TEST_IMAGE(PATH_TO_PACKAGE "/data/nadezhda_diskant.jpg");

// Test with calls to timer_callback that publishes the image as sensor_msgs
TEST(ImageViewer_TestSuite, Test_ImageViewer_)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + " input_data").c_str());

  auto imageviewer_node = std::make_shared<ImageViewer>();

  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  imageviewer_node->activate_image_callback(input_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
