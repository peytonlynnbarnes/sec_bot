# SEC Bot - Docker Deployment with Micro-ROS

## Prerequisites

- Docker
- Docker Compose (optional, but recommended)

## Quick Start

### Build the Docker Image

```bash
sudo DOCKER_BUILDKIT=1 docker build --memory 6g -t sec_bot .
```

# If you are using wsl, increase memory available:
```bash
sudo fallocate -l 12G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**Note:** Using `DOCKER_BUILDKIT=1` enables advanced caching and build features.

### Run the Container

#### Basic Interactive Shell

```bash
docker run -it --privileged \
    --network host \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    sec_bot
```

#### Run Micro-ROS Agent

```bash
# Replace /dev/ttyACM0 with your microcontroller's serial port
docker run -it --privileged \
    --network host \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    sec_bot \
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## Advanced Configuration

### Docker Compose

Create a `docker-compose.yml`:

```yaml
version: "3"
services:
  sec_bot:
    build: .
    privileged: true
    network_mode: host
    volumes:
      - /dev:/dev
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=$DISPLAY
    devices:
      - "/dev/ttyACM0:/dev/ttyACM0" # Add microcontroller device
```

Run with:

```bash
docker-compose up
```

## Accessing the Container

- The container starts an interactive bash shell
- Source ROS2 setup: `source /opt/ros/jazzy/setup.bash`
- Source micro-ROS setup: `source /microros_ws/install/local_setup.bash`
- Source project setup: `source /sec_bot/install/setup.bash`

## Micro-ROS Commands

Inside the container, run micro-ROS components:

```bash
# Start Micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# List available micro-ROS nodes
ros2 run micro_ros_setup list_nodes
```

## Simulation Commands

Inside the container, run simulation components:

```bash
# Terminal 1: Gazebo Simulation
ros2 launch sec_bot launch_sim.launch.py

# Terminal 2: Camera Subscriber
ros2 launch ball_tracker sim_cam.launch.py


# Terminal 3: Ball Tracking
ros2 run ball_tracker sim_multi_ball_tracker

# Terminal 4: Ball Following
ros2 launch ball_tracker follow_ball.launch.py
```

## Troubleshooting

- Ensure X11 forwarding is enabled
- Check Docker and container permissions
- Verify host system graphics drivers
- Confirm microcontroller serial port connection
- Verify micro-ROS agent communication

## Micro-ROS Development

- The Docker image includes full micro-ROS setup
- Develop and build micro-ROS applications in the `/microros_ws` directory
- Use `colcon build` to compile micro-ROS packages
