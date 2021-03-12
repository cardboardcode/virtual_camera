#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include <chrono>
#include <string>
#include <functional>
#include <memory>
#include <string>

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

TEST(VirtualCamera_TestSuite, Test_VirtualCamera_ImageInputData)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();
}

TEST(VirtualCamera_TestSuite, Test_VirtualCamera_VideoInputData)
{
  system(("ln -sf " + PATH_TO_TEST_VIDEO + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();
}

TEST(VirtualCamera_TestSuite, Test_VirtualCamera_AdvanceCursor)
{
  system(("ln -sf " + PATH_TO_TEST_IMAGE + PATH_TO_INPUT_DATA).c_str());

  auto vcamera_node = std::make_shared<VirtualCamera>();

  vcamera_node->activate_advance_cursor();
}


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
