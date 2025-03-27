# syntax = docker/dockerfile:1.2
FROM ubuntu:24.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set up cache directories
ENV PIP_CACHE_DIR=/var/cache/buildkit/pip
ENV MICROROS_WS=/microros_ws
ENV PANGOLIN_CACHE=/pangolin_cache
ENV ORB_SLAM3_CACHE=/orb_slam3_cache
ENV ROS_WS=/ros2_ws

# Create necessary directories
RUN mkdir -p $PIP_CACHE_DIR $MICROROS_WS $PANGOLIN_CACHE $ORB_SLAM3_CACHE $ROS_WS

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
    python3-vcstool \
    libglew-dev \
    libxkbcommon-dev \
    libwayland-dev \
    libglu1-mesa-dev

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

# Install additional dependencies
RUN --mount=type=cache,target=$PIP_CACHE_DIR \
    apt-get update && \
    apt-get install -y \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-pip \
    && \
    . /opt/venv/bin/activate && \
    pip install --upgrade pip setuptools wheel && \
    pip install \
    catkin-pkg \
    rosdep \
    vcstool \
    colcon-common-extensions \
    rospkg \
    empy \
    numpy \
    opencv-python

# Prepare ROS2 workspace
WORKDIR $ROS_WS

# Create separate build stage for ros2_orb_slam3 - will only rebuild if source changes
RUN mkdir -p src/ros2_orb_slam3
COPY src/ros2_orb_slam3 src/ros2_orb_slam3/
RUN --mount=type=cache,target=$ROS_WS/src/ros2_orb_slam3/build \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select ros2_orb_slam3"

# Install camera dependencies (after orbslam, so won't take a long time to build)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    v4l-utils \
    libv4l-dev \
    ros-jazzy-v4l2-camera \
    python3-opencv \
    libcap-dev \
    libopencv-dev

RUN --mount=type=cache,target=$PIP_CACHE_DIR \
    . /opt/venv/bin/activate && \
    pip install \
    v4l2-python3 \
    picamera2 \
    opencv-python

# Fix NumPy version and rebuild compatibility
RUN --mount=type=cache,target=$PIP_CACHE_DIR \
    . /opt/venv/bin/activate && \
    pip uninstall -y numpy && \
    pip install "numpy<2.0" pybind11>=2.12 && \
    pip install opencv-python-headless

# Reinstall or rebuild NumPy-dependent packages
RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source $ROS_WS/install/setup.bash && \
    pip install --upgrade numpy opencv-python && \
    pip install --upgrade cv-bridge && \
    colcon build --packages-select ball_tracker --cmake-clean-cache"

# Copy and build other packages that change more frequently
COPY src/ball_tracker src/ball_tracker/
COPY src/sec_bot src/sec_bot/

# Build remaining packages
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    source $ROS_WS/install/setup.bash && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy && \
    colcon build --symlink-install --packages-select ball_tracker sec_bot"

# Create an entrypoint script to source ROS2 setup
RUN echo '#!/bin/bash\n\
    source /opt/ros/jazzy/setup.bash\n\
    source $ROS_WS/install/setup.bash\n\
    exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set up entrypoint and default command
COPY entrypoint.sh /custom_entrypoint.sh
RUN chmod +x /custom_entrypoint.sh

# Set default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]