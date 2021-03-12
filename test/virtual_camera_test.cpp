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

bool isImagePublished = false;
bool isShuttingDown = false;

bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

TEST(VirtualCamera_TestSuite, Test1)
{
  EXPECT_TRUE(isImagePublished);
}

TEST(VirtualCamera_TestSuite, Test2)
{
  system("rm test_image.jpg");
  system("rm data/random_video");
  EXPECT_TRUE(isShuttingDown);
}

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("test_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("/virtual_camera/state_input", 10);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "shutdown";
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  if (!is_file_exist("./data/9544757988_991457c228_z.jpg")) {
    system("apt-get install -y curl");
    system("curl https://farm8.staticflickr.com/7329/9544757988_991457c228_z.jpg --output test_image.jpg");
    system("ln -f test_image.jpg random_video");
    system("mkdir data");
    system("mv random_video data/random_video");
  }

  auto camera_node = std::make_shared<VirtualCamera>();

  auto pub_node = std::make_shared<MinimalPublisher>();

  auto sub_node = rclcpp::Node::make_shared("test_subscriber");

  // size_t depth_ = rmw_qos_profile_default.depth;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy_, depth_));
  qos.reliability(reliability_policy_);

  auto callback = [sub_node](const sensor_msgs::msg::Image::SharedPtr msg)
  {
    isImagePublished = true;
  };

  auto test_result_sub = sub_node->create_subscription<sensor_msgs::msg::Image>("/virtual_camera/image_raw",
    qos, callback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);
  executor.add_node(camera_node);
  executor.add_node(pub_node);

  rclcpp::WallRate loop_rate(50);
  for (int i = 0; i < 300; ++i) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }
    executor.spin_once();
    // printf(camera_node->shuttingDown ? "true\n" : "false\n");
    isShuttingDown = camera_node->shuttingDown;
  }
  return RUN_ALL_TESTS();
}
