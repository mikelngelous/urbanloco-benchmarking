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
    tcpdump \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy source code
COPY src/ src/

# Build the workspace (skip rosdep install due to network issues)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Production stage
FROM base as prod

# Copy built workspace
COPY --from=base $CATKIN_WS/devel $CATKIN_WS/devel
COPY --from=base $CATKIN_WS/build $CATKIN_WS/build

# Set up entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
