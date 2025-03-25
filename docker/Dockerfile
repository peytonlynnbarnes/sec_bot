# Use Ubuntu 24.04 as base image
FROM ubuntu:24.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /sec_bot

# Install system dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    software-properties-common \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    pipx \
    lsb-release \
    gnupg \
    libepoxy-dev

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Gazebo repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS2 Jazzy and dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    python3-opencv \
    libopencv-dev \
    libglew-dev \
    libpython3-dev \
    python3-numpy \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libeigen3-dev

# Install pipx tools
RUN pipx install wheel && \
    pipx ensurepath

# Clone and build Pangolin
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Clone SEC Bot repository
RUN git clone https://github.com/peytonlynnbarnes/sec_bot.git --branch jazzy-stuff

# Set up working directory
WORKDIR /sec_bot

# Download ORB SLAM Vocabulary
RUN cd src/ball_tracker/config/Vocabulary && \
    wget https://github.com/raulmur/ORB_SLAM2/raw/refs/heads/master/Vocabulary/ORBvoc.txt.tar.gz && \
    tar xf ORBvoc.txt.tar.gz && \
    rm -rf ORBvoc.txt.tar.gz

# Create symbolic link for OpenCV
RUN ln -sf /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0 /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5d

# Source ROS2 setup and build the project
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Set up entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set default command
ENTRYPOINT ["/entrypoint.sh"]