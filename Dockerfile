FROM ros:jazzy-perception-noble

# Add user
RUN adduser --quiet --disabled-password user

ENV DEBIAN_FRONTEND=noninteractive
# Install OpenCV
RUN apt-get update && \
    apt-get install -y xauth xxd x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/user/virtual_camera_ws
RUN mkdir src
COPY . src/virtual_camera

# colcon compilation
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN sed -i '$isource "/home/user/virtual_camera_ws/install/setup.bash"' /ros_entrypoint.sh

RUN chown -R user:user /home/user/
USER user

ENTRYPOINT ["/ros_entrypoint.sh"]
