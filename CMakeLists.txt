cmake_minimum_required(VERSION 3.5)
project(virtual_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(virtual_camera_test_cpp test/virtual_camera_test.cpp)

  ament_target_dependencies(virtual_camera_test_cpp rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
endif()

add_definitions(-DPATH_TO_PACKAGE="${CMAKE_CURRENT_LIST_DIR}")

add_executable(virtual_camera src/virtual_camera.cpp)
ament_target_dependencies(virtual_camera rclcpp std_msgs sensor_msgs OpenCV cv_bridge ament_index_cpp)

add_executable(image_viewer src/image_viewer.cpp)
ament_target_dependencies(image_viewer rclcpp std_msgs sensor_msgs OpenCV cv_bridge)


install(TARGETS
  virtual_camera
  image_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
