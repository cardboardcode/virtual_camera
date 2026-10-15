ARG ROS_DISTRO=lyrical
FROM ros:$ROS_DISTRO-perception-resolute
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 packages and build dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rosbridge-server \
    curl \
    build-essential \
    pkg-config \
    libssl-dev \
    libclang-dev \
    clang \
    xvfb \
    xauth \
    xxd \
    x11-xserver-utils && \
    rm -rf /var/lib/apt/lists/*

# Install Rust toolchain
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH \
    CPLUS_INCLUDE_PATH="/usr/include/c++/15:/usr/include/x86_64-linux-gnu/c++/15" \
    OPENCV_CLANG_ARGS="--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/15"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable --profile minimal \
    && chmod -R a+w /usr/local/cargo

WORKDIR /virtual_camera_ws/src/virtual_camera
COPY . .

# Build virtual_camera Rust binaries
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && cargo build --release

# Install binaries into standard system and ROS paths, and register into ROS 2 ament index
RUN mkdir -p /usr/local/bin \
    && cp target/release/virtual_camera /usr/local/bin/ \
    && cp target/release/image_viewer /usr/local/bin/ \
    && mkdir -p /opt/ros/$ROS_DISTRO/lib/virtual_camera \
    && cp target/release/virtual_camera /opt/ros/$ROS_DISTRO/lib/virtual_camera/ \
    && cp target/release/image_viewer /opt/ros/$ROS_DISTRO/lib/virtual_camera/ \
    && mkdir -p /opt/ros/$ROS_DISTRO/share/virtual_camera/launch \
    && cp launch/showimageraw.launch.py /opt/ros/$ROS_DISTRO/share/virtual_camera/launch/ \
    && cp package.xml /opt/ros/$ROS_DISTRO/share/virtual_camera/ \
    && mkdir -p /opt/ros/$ROS_DISTRO/share/ament_index/resource_index/packages \
    && touch /opt/ros/$ROS_DISTRO/share/ament_index/resource_index/packages/virtual_camera \
    && mkdir -p /virtual_camera_ws/install/virtual_camera/lib/virtual_camera \
    && cp target/release/virtual_camera /virtual_camera_ws/install/virtual_camera/lib/virtual_camera/ \
    && cp target/release/image_viewer /virtual_camera_ws/install/virtual_camera/lib/virtual_camera/ \
    && mkdir -p /virtual_camera_ws/install/virtual_camera/share/virtual_camera/launch \
    && cp launch/showimageraw.launch.py /virtual_camera_ws/install/virtual_camera/share/virtual_camera/launch/ \
    && cp package.xml /virtual_camera_ws/install/virtual_camera/share/virtual_camera/ \
    && mkdir -p /virtual_camera_ws/install/virtual_camera/share/ament_index/resource_index/packages \
    && touch /virtual_camera_ws/install/virtual_camera/share/ament_index/resource_index/packages/virtual_camera

RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"$ROS_DISTRO"'/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
