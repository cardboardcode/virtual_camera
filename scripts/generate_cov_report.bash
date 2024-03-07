
# Source local ROS2 distro
source /opt/ros/humble/setup.bash

if [[ $1 == "update" ]]; then
  sudo rm -r build/ install/ log/ lcov/
  colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'
  colcon lcov-result --initial
  colcon test --packages-select virtual_camera
  colcon lcov-result
elif [[ $1 == "clean" ]]; then
  sudo rm -r build/ install/ log/ lcov/
else
  colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage'
  colcon lcov-result --initial
  colcon test --packages-select virtual_camera
  colcon lcov-result
fi
