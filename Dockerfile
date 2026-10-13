ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO-perception-noble
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
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable --profile minimal \
    && chmod -R a+w /usr/local/cargo

WORKDIR /virtual_camera_ws/src/virtual_camera
COPY . .

# Build virtual_camera Rust binaries
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && cargo build --release

# Install binaries into standard executable paths
RUN mkdir -p /virtual_camera_ws/install/virtual_camera/lib/virtual_camera \
    && cp target/release/virtual_camera /usr/local/bin/ \
    && cp target/release/image_viewer /usr/local/bin/ \
    && cp target/release/virtual_camera /virtual_camera_ws/install/virtual_camera/lib/virtual_camera/ \
    && cp target/release/image_viewer /virtual_camera_ws/install/virtual_camera/lib/virtual_camera/

RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"$ROS_DISTRO"'/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
