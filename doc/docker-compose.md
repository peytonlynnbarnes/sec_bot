# Docker Compose Workflow for ROS2 Sec Bot Project

## Initial Docker Setup and Permissions

### Setting Up Docker Without Sudo
```bash
# Create docker group if it doesn't exist
sudo groupadd docker

# Add your user to the docker group
sudo usermod -aG docker $USER

# Apply group membership
newgrp docker

# Ensure Docker daemon starts on boot
sudo systemctl enable docker
```

### Verify Docker Installation
```bash
# Check Docker version
docker --version

# Test Docker without sudo
docker run hello-world
```

## Workflow Commands

### Accessing the Container
```bash
# Open main terminal
docker-compose exec sec_bot /bin/bash

# Open additional terminals
docker-compose exec sec_bot /bin/bash
```

### Persistent Modifications
1. Install Packages Inside Container:
```bash
# Inside container
sudo apt-get update
sudo apt-get install new-package
```

2. Edit Source Code:
```bash
# Edit files directly in the mounted volume
nano /sec_bot/src/ball_tracker/some_file.py
```

### Multi-Terminal Simulation Workflow
Terminal 1: Start Container and Gazebo
```bash
docker-compose exec sec_bot \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 launch sec_bot launch_sim.launch.py"
```

Terminal 2: Camera Subscriber
```bash
docker-compose exec sec_bot \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 launch ball_tracker sim_cam.launch.py"
```

Terminal 3: Ball Tracking
```bash
docker-compose exec sec_bot \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 run ball_tracker sim_multi_ball_tracker"
```

Terminal 4: Ball Following
```bash
docker-compose exec sec_bot \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 launch ball_tracker follow_ball.launch.py"
```

## Managing the Container

### Build and Initial Setup
```bash
# Build the image and create volumes
docker-compose build

# Start the container
docker-compose up -d
```

### Stop Container
```bash
docker-compose down
```

### Restart Container
```bash
docker-compose up -d
```

### Rebuild with Changes
```bash
docker-compose build
docker-compose up -d
```

## Micro-ROS Commands
```bash
# Inside container
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run micro_ros_setup list_nodes
```

## Additional Tips
- All terminals share the same `/ros2_ws/src` volume
- Changes persist between sessions
- Easy to rebuild and reset if needed

## Troubleshooting
- Ensure X11 forwarding is enabled
- Check ROS_DOMAIN_ID for network consistency
- Verify volume mounts
- If permission issues persist, recheck Docker group setup

## Interactive Container Setup
```bash
# Recommended interactive setup
docker-compose exec sec_bot bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    /bin/bash"
```