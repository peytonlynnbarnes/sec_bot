# syntax = docker/dockerfile:1.2
FROM ubuntu:24.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set up cache directories
ENV PIP_CACHE_DIR=/var/cache/buildkit/pip
ENV MICROROS_WS=/microros_ws
ENV PANGOLIN_CACHE=/pangolin_cache
ENV ORB_SLAM_VOCAB_CACHE=/orb_slam_vocab_cache

# Create necessary directories
RUN mkdir -p $PIP_CACHE_DIR $MICROROS_WS $PANGOLIN_CACHE $ORB_SLAM_VOCAB_CACHE

# Remove Docker's default apt-get cleanup configuration
RUN rm -f /etc/apt/apt.conf.d/docker-clean

# Set working directory
WORKDIR /sec_bot

# Stage 1: System dependencies and repositories
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    sudo \
    software-properties-common \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    lsb-release \
    gnupg \
    libepoxy-dev \
    python3-pip \
    python3-venv \
    python3-full \
    pipx

# Add ROS2 repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Gazebo repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS2 Jazzy and dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
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
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-vcstool

# Create a virtual environment and install pip tools
RUN python3 -m venv /opt/venv && \
    . /opt/venv/bin/activate && \
    pip install --cache-dir=$PIP_CACHE_DIR wheel setuptools pipx && \
    pipx ensurepath

# Set up PATH to include virtual environment
ENV PATH="/opt/venv/bin:$PATH"

# Prepare micro-ROS workspace
WORKDIR $MICROROS_WS

# Fetch micro-ROS dependencies with caching
RUN --mount=type=cache,id=microros-cache,target=/microros_ws/src,sharing=locked \
   # git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    #vcs import src < src/micro_ros_setup/dependencies.repos && \
    apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools with caching
RUN --mount=type=cache,id=microros-build-cache,target=/microros_ws/build \
    . /opt/ros/jazzy/setup.sh && \
    colcon build && \
    . install/local_setup.sh

# Prepare Pangolin cache
WORKDIR $PANGOLIN_CACHE
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

# Switch back to sec_bot workspace
WORKDIR /sec_bot

# Build Pangolin from cached source
RUN cp -R $PANGOLIN_CACHE/Pangolin . && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Prepare ORB SLAM Vocabulary cache
WORKDIR $ORB_SLAM_VOCAB_CACHE
RUN wget https://github.com/raulmur/ORB_SLAM2/raw/refs/heads/master/Vocabulary/ORBvoc.txt.tar.gz && \
    tar xf ORBvoc.txt.tar.gz

# Create symbolic link for OpenCV
RUN find /usr/lib -name "libopencv_core.so*"
RUN ls /usr/lib/x86_64-linux-gnu/libopencv_core.so*
RUN ln -sf /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0 /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5d

# Copy local repository into the container
WORKDIR /sec_bot
COPY . /sec_bot/

# Copy cached ORB SLAM Vocabulary
RUN mkdir -p src/ball_tracker/config/Vocabulary && \
    cp $ORB_SLAM_VOCAB_CACHE/ORBvoc.txt src/ball_tracker/config/Vocabulary/



# Source ROS2 and micro-ROS setups, then build the project
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    source $MICROROS_WS/install/local_setup.bash && \
    colcon build --symlink-install"

# Set up entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set default command
ENTRYPOINT ["/entrypoint.sh"]