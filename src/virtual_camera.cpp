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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "virtual_camera/virtual_camera.hpp"
#include "virtual_camera/image_viewer.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  // Connect the nodes as a pipeline: vcamera_node -> image_view_node
  // Instantiate VirtualCamera ROS 2 node.
  std::shared_ptr<VirtualCamera> vcamera_node = nullptr;
  try {
    vcamera_node = std::make_shared<VirtualCamera>();
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting ..\n", e.what());
    return 1;
  }

  // Declare a boolean parameter
  vcamera_node->declare_parameter("use_image_viewer", false);

  // Get the boolean parameter value to determine if the image_view_node should be activated.
  bool imageviewer_flag = vcamera_node->get_parameter("use_image_viewer").as_bool();
  RCLCPP_INFO(vcamera_node->get_logger(), "use_image_viewer set to: [%s]", imageviewer_flag ? "true" : "false");

  // Instantiate ImageViewer ROS 2 node.
  auto image_view_node = std::make_shared<ImageViewer>();
  // Add VirtualCamera ROS 2 node to intra-process executor
  executor.add_node(vcamera_node);

  // If use_image_viewer ROS 2 parameter is set to true at launch time, 
  // add ImageViewer ROS 2 node to intra-process executor.
  if (imageviewer_flag) {
    executor.add_node(image_view_node);
  }

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
