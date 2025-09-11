# Multi-stage build for ROS1 Robot Communication System
FROM ros:noetic-perception as base

# Set environment variables
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/catkin_ws
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    libboost-all-dev \
    protobuf-compiler \
    libprotobuf-dev \
    liblz4-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    pkg-config \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep (skip init as it's already done in base image)
RUN rosdep update

# Create catkin workspace
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy source code
COPY src/ src/

# Install ROS dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Development stage
FROM base as dev

# Install development tools
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    clang-tidy \
    cppcheck \
    vim \
    tmux \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Production stage
FROM base as prod

# Copy built workspace
COPY --from=base $CATKIN_WS/devel $CATKIN_WS/devel
COPY --from=base $CATKIN_WS/build $CATKIN_WS/build

# Set up entrypoint for production
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "robot_communication", "robot_communication.launch"]