#!/bin/bash
set -e

# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Source ROS 2 installation
source /microros_ws/install/setup.bash

# Source micro-ROS workspace
source /microros_ws/install/local_setup.bash

# If no command is provided, start a bash shell
if [ $# -eq 0 ]; then
    exec bash
else
    # Otherwise, execute the provided command
    exec "$@"
fi