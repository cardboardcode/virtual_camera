ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO-perception-noble
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    xauth \
    xxd \
    x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /virtual_camera_ws
RUN mkdir src
COPY . src/virtual_camera

# Build virtual_camera ROS 2 package
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN sed -i '$isource "/virtual_camera_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]