# Target ROS 2 distro. Plain `ros:<distro>-perception` tag tracks the current
# Ubuntu codename for that distro (e.g. jazzy -> noble, rolling -> resolute),
# so a single knob covers Jazzy and Rolling.
ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-perception
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

# Build virtual_camera Rust binaries.
#
# The `opencv` crate's binding generator (opencv-binding-generator) drives
# libclang directly, and its C include search paths are taken from
# `clang_sys::support::Clang` rather than the real g++ search path. On
# Rolling (resolute / GCC-15) clang-21's built-in paths point at a GCC-16
# location that does not exist, so it fails to find C++ stdlib headers
# (e.g. `bits/c++config.h`) and C headers (`stdlib.h`). The generator honours
# the `OPENCV_CLANG_ARGS` env var (generator.rs:435) for extra include dirs, so
# we feed it the *complete* standard include chain as reported by `g++ -v`
# (covers both /usr/include/c++/NN and the arch-specific
# /usr/include/<arch>/c++/NN, plus the C header roots). This is a no-op on
# Jazzy/noble where clang-18 already resolves those paths.
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && ISYSTEMS=$(g++ -x c++ -v -fsyntax-only - </dev/null 2>&1 \
         | awk '/#include <...> search starts here:/{f=1;next} /^ *[^ ]+#include/{f=0} f && /^ *\//{print}') \
    && OPENCV_CLANG_ARGS=$(printf '%s\n' "$ISYSTEMS" | sed -e 's/^[[:space:]]*//' -e 's|^|-isystem|' | tr '\n' ' ') \
    && echo "OPENCV_CLANG_ARGS=${OPENCV_CLANG_ARGS}" \
    && export OPENCV_CLANG_ARGS \
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

# Entrypoint: source the installed ROS distro (image is built for exactly one).
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'if [ -z "${ROS_DISTRO:-}" ]; then' \
    '    for d in rolling jazzy; do [ -d "/opt/ros/$d" ] && export ROS_DISTRO=$d && break; done' \
    '    [ -n "${ROS_DISTRO:-}" ] || { echo "No ROS distro found in /opt/ros" >&2; exit 1; }' \
    'fi' \
    '. /opt/ros/$ROS_DISTRO/setup.bash' \
    'exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
