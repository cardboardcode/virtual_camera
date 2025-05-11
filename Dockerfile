ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

# Add user
RUN adduser --quiet --disabled-password user
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    tmux \
    curl \
    clang \
    python3-pip \
    libclang-dev \
    python3-vcstool \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install git+https://github.com/colcon/colcon-cargo.git
RUN pip install git+https://github.com/colcon/colcon-ros-cargo.git


# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.82.0 -y
ENV PATH=/root/.cargo/bin:$PATH

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN if [ "$ROS_DISTRO" = "humble" ] ;  \
    then pip install --upgrade pytest && pip install colcon-ros-cargo ;  \
    else pip install --break-system-packages pytest colcon-ros-cargo ; fi

WORKDIR /workspace
RUN mkdir src
RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust --depth 1 --branch main
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos

# colcon compilation 1
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build

RUN mkdir -p src/virtual_camera
WORKDIR /workspace/src/virtual_camera
COPY launch launch
COPY src src
COPY Cargo.toml Cargo.toml
COPY package.xml package.xml
WORKDIR /workspace

# colcon compilation 2
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --packages-select virtual_camera

# cleanup
RUN sed -i '$isource "/workspace/install/setup.bash"' /ros_entrypoint.sh

RUN chown -R user:user /workspace/
USER user

ENTRYPOINT ["/ros_entrypoint.sh"]

