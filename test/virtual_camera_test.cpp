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
#include "virtual_camera/virtual_camera.hpp"

using namespace std::chrono_literals;

std::string PATH_TO_TEST_IMAGE(PATH_TO_PACKAGE "/data/nadezhda_diskant.jpg");
std::string PATH_TO_TEST_VIDEO(PATH_TO_PACKAGE "/data/video.mp4");
std::string PATH_TO_INPUT_DATA(PATH_TO_PACKAGE "/data/input_data");

// Test with a static image as input_data.
TEST(VirtualCamera_TestSuite, Test_VirtualCamera_ImageInputData)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();
}

// Test with a static video as input_data.
TEST(VirtualCamera_TestSuite, Test_VirtualCamera_VideoInputData)
{
  system(("ln -sf " + PATH_TO_TEST_VIDEO + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();
}

// Test with calls to advance cursor.
TEST(VirtualCamera_TestSuite, Test_VirtualCamera_AdvanceCursor)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();

  vcamera_node->activate_advance_cursor();
}

// Test with calls to timer_callback that publishes the image as sensor_msgs
TEST(VirtualCamera_TestSuite, Test_VirtualCamera_TimerCallback)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + " input_data").c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();

  vcamera_node->activate_time_callback();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
