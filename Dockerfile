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
# libclang directly, and its C++ include search paths are taken from
# `clang_sys::support::Clang` (`clang -E -x c++ - -v`). On Rolling (Ubuntu
# Resolute / GCC 15), `libgcc-16-dev` is also present in the base image (via
# gfortran-16), causing clang-21 to select GCC 16 even though `libstdc++-16-dev`
# is not installed; consequently clang omits the C++ stdlib include dirs.
# Passing those dirs via `OPENCV_CLANG_ARGS` as `-isystem` fails because
# `opencv-binding-generator` appends `OPENCV_CLANG_ARGS` *after* `-isystem/usr/include`,
# breaking `#include_next <stdlib.h>` inside `<cstdlib>`. Setting
# `CPLUS_INCLUDE_PATH` to the C++ stdlib dirs reported by `g++ -v` causes clang
# to prepend them as `-cxx-isystem` *before* `/usr/include`.
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && CPLUS_INCLUDE_PATH=$(g++ -x c++ -v -fsyntax-only - </dev/null 2>&1 \
         | awk '/#include <...> search starts here:/{f=1;next} /End of search list/{f=0} f && /c\+\+/{gsub(/^[ \t]+/, ""); print}' \
         | paste -sd: -) \
    && echo "CPLUS_INCLUDE_PATH=${CPLUS_INCLUDE_PATH}" \
    && export CPLUS_INCLUDE_PATH \
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

# Entrypoint: source ROS 2 Rolling and configure CPLUS_INCLUDE_PATH for any in-container cargo builds/tests.
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'if [ -z "${CPLUS_INCLUDE_PATH:-}" ] && command -v g++ >/dev/null 2>&1; then' \
    '    export CPLUS_INCLUDE_PATH=$(g++ -x c++ -v -fsyntax-only - </dev/null 2>&1 | awk '\''/#include <...> search starts here:/{f=1;next} /End of search list/{f=0} f && /c\+\+/{gsub(/^[ \t]+/, ""); print}'\'' | paste -sd: -)' \
    'fi' \
    'source /opt/ros/'"$ROS_DISTRO"'/setup.bash' \
    'exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
