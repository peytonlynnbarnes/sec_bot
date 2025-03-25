#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /sec_bot/install/setup.bash

# Default command (can be overridden)
if [ $# -eq 0 ]; then
    echo "Container started. Use docker exec to run specific ROS2 commands."
    tail -f /dev/null  # Keep container running
else
    exec "$@"
fi
