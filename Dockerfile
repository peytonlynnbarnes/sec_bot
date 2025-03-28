# syntax = docker/dockerfile:1.2
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set up cache directories
ENV PIP_CACHE_DIR=/var/cache/buildkit/pip
ENV MICROROS_WS=/microros_ws
ENV PANGOLIN_CACHE=/pangolin_cache
ENV ORB_SLAM3_CACHE=/orb_slam3_cache
ENV ROS_WS=/ros2_ws

# Set up X11 display environment variables
ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
ENV QT_GRAPHICSSYSTEM=native
ENV NO_AT_BRIDGE=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV QT_QPA_PLATFORM=xcb
ENV QT_QPA_PLATFORMTHEME=gtk2
ENV QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms

# Create necessary directories
RUN mkdir -p $PIP_CACHE_DIR $MICROROS_WS $PANGOLIN_CACHE $ORB_SLAM3_CACHE $ROS_WS /tmp/runtime-root
RUN chmod 700 /tmp/runtime-root

# Remove Docker's default apt-get cleanup configuration
RUN rm -f /etc/apt/apt.conf.d/docker-clean

# Set working directory
WORKDIR /sec_bot

# Configure APT to be more resilient to network issues
RUN echo "Acquire::Retries \"10\";" > /etc/apt/apt.conf.d/80-retries && \
    echo "APT::Get::Assume-Yes \"true\";" > /etc/apt/apt.conf.d/90-assume-yes && \
    echo "APT::Install-Recommends \"false\";" > /etc/apt/apt.conf.d/99-no-recommends

# Stage 1: System dependencies and repositories
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
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
    pipx \
    usbutils \
    bison \
    flex \
    libfl-dev \
    libasio-dev \
    # X11 and Qt dependencies
    x11-apps \
    libxcb1-dev \
    libxcb-keysyms1-dev \
    libxcb-image0-dev \
    libxcb-icccm4-dev \
    libxcb-render-util0-dev \
    libxcb-randr0-dev \
    libxcb-xinerama0-dev \
    libxkbcommon-x11-dev \
    libxcb-shape0-dev \
    libxcb-xfixes0-dev \
    libqt5gui5 \
    qtbase5-dev \
    mesa-utils

# Add ROS2 repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Gazebo repository
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS2 Humble and dependencies (changed from Jazzy)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-xacro \
    ros-humble-ros-gz \
    python3-opencv \
    libopencv-dev \
    libglew-dev \
    libpython3-dev \
    python3-numpy \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python3-colcon-common-extensions \
    python3-vcstool \
    libglew-dev \
    libxkbcommon-dev \
    libwayland-dev \
    libglu1-mesa-dev \
    ros-humble-cv-bridge \
    python3-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-osrf-testing-tools-cpp \
    ros-humble-test-interface-files

# Create a virtual environment and install pip tools
RUN python3 -m venv /opt/venv && \
    . /opt/venv/bin/activate && \
    pip install --cache-dir=$PIP_CACHE_DIR wheel setuptools pipx && \
    pipx ensurepath

# Set environment variable to find ROS headers
ENV CPATH=/opt/ros/humble/include:$CPATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/ros/humble:$CMAKE_PREFIX_PATH

# Ensure cv_bridge headers are available system-wide
RUN mkdir -p /usr/include/cv_bridge && \
    cp -r /opt/ros/humble/include/cv_bridge/* /usr/include/cv_bridge/ && \
    ln -sf /opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.h /usr/include/cv_bridge/cv_bridge.hpp

# Prepare ROS2 workspace
WORKDIR $ROS_WS

# Stage 6: Install micro-ROS - pre-install dependencies to avoid network issues
RUN apt-get update && apt-get install -y --no-install-recommends \
    clang \
    clang-tidy \
    clang-format \
    liblog4cxx-dev \
    libasio-dev \
    usbutils

# Initialize rosdep before updating dependencies
RUN sudo rosdep init && \
    rosdep update

# Create micro-ROS workspace and clone repository
WORKDIR $MICROROS_WS
RUN mkdir -p src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Build micro-ROS tools and source them
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd $MICROROS_WS && \
    colcon build && \
    source install/local_setup.bash"

# Create firmware workspace with explicit host platform
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source $MICROROS_WS/install/local_setup.bash && \
    cd $MICROROS_WS && \
    ros2 run micro_ros_setup create_firmware_ws.sh host || exit_code=$? && \
    if [ ! -z \"\$exit_code\" ] && [ \"\$exit_code\" -ne 0 ]; then \
        echo 'Error creating firmware workspace, trying again...' && \
        apt-get update && \
        apt-get install -y --fix-missing usbutils libasio-dev && \
        ros2 run micro_ros_setup create_firmware_ws.sh host; \
    fi"

# Build firmware with more verbose output and error handling
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source $MICROROS_WS/install/local_setup.bash && \
    cd $MICROROS_WS && \
    ros2 run micro_ros_setup build_firmware.sh || exit_code=$? && \
    if [ ! -z \"\$exit_code\" ] && [ \"\$exit_code\" -ne 0 ]; then \
        echo 'Error building firmware, trying again...' && \
        apt-get update && \
        apt-get install -y --fix-missing && \
        ros2 run micro_ros_setup build_firmware.sh; \
    fi"

# Create micro-ROS agent workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source $MICROROS_WS/install/local_setup.bash && \
    cd $MICROROS_WS && \
    ros2 run micro_ros_setup create_agent_ws.sh || exit_code=$? && \
    if [ ! -z \"\$exit_code\" ] && [ \"\$exit_code\" -ne 0 ]; then \
        echo 'Error creating agent workspace, trying again...' && \
        apt-get update && \
        apt-get install -y --fix-missing && \
        ros2 run micro_ros_setup create_agent_ws.sh; \
    fi"

# Build micro-ROS agent with verbose output
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source $MICROROS_WS/install/local_setup.bash && \
    cd $MICROROS_WS && \
    ros2 run micro_ros_setup build_agent.sh || exit_code=$? && \
    if [ ! -z \"\$exit_code\" ] && [ \"\$exit_code\" -ne 0 ]; then \
        echo 'Error building agent, trying again...' && \
        apt-get update && \
        apt-get install -y --fix-missing && \
        ros2 run micro_ros_setup build_agent.sh; \
    fi"

# Stage 7: more dependencies
# Install camera dependencies (after orbslam, so won't take a long time to build)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    v4l-utils \
    libv4l-dev \
    ros-humble-v4l2-camera \
    python3-opencv \
    libcap-dev \
    libopencv-dev \
    # Additional Gazebo X11 fix packages
    libglu1-mesa \
    libegl1-mesa \
    libegl1 \
    xvfb

RUN --mount=type=cache,target=$PIP_CACHE_DIR \
    . /opt/venv/bin/activate && \
    pip install \
    v4l2-python3 \
    picamera2 \
    opencv-python

# Stage 8: numpy version compability - FIX: Remove sourcing of non-existent setup.bash
# Fix NumPy version, rebuild compatibility, and ensure dependent modules are consistent
RUN --mount=type=cache,target=$PIP_CACHE_DIR \
    . /opt/venv/bin/activate && \
    pip uninstall -y numpy && \
    pip install --cache-dir=$PIP_CACHE_DIR "numpy<2.0" pybind11>=2.12 opencv-python opencv-python-headless cv-bridge

# Stage 9: Copy source files first
# Copy source files before building
WORKDIR $ROS_WS
COPY src/ball_tracker src/ball_tracker/
COPY src/sec_bot src/sec_bot/

# Stage 10: Build the packages - now that source code is available
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export CPLUS_INCLUDE_PATH=/opt/ros/humble/include:$CPLUS_INCLUDE_PATH && \
    cd $ROS_WS && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro humble && \
    colcon build --symlink-install --packages-select ball_tracker sec_bot --cmake-args \
    -DCMAKE_CXX_FLAGS='-I/opt/ros/humble/include -I/usr/include/eigen3'"

# Create Xvfb startup script for headless systems
RUN echo '#!/bin/bash\n\
# Start Xvfb\n\
Xvfb :1 -screen 0 1280x1024x24 &\n\
sleep 2\n\
\n\
# Source ROS environments\n\
source /opt/ros/humble/setup.bash\n\
source $MICROROS_WS/install/local_setup.bash\n\
if [ -f "$ROS_WS/install/local_setup.bash" ]; then\n\
  source $ROS_WS/install/local_setup.bash\n\
fi\n\
\n\
# Execute the command passed to the script\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

# Add a healthcheck to verify the display server is working
HEALTHCHECK --interval=30s --timeout=10s --start-period=15s --retries=3 \
  CMD xdpyinfo -display :1 >/dev/null 2>&1 || exit 1

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
