# SEC Bot - Docker Deployment

## Prerequisites
- Docker
- Docker Compose (optional, but recommended)

## Quick Start

### Build the Docker Image
```bash
docker build -t sec_bot .
```

### Run the Container
```bash
docker run -it --privileged \
    --network host \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    sec_bot
```

## Advanced Configuration

### Docker Compose
Create a `docker-compose.yml`:
```yaml
version: '3'
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
```

Run with:
```bash
docker-compose up
```

## Accessing the Container
- The container starts an interactive bash shell
- Source ROS2 setup: `source /opt/ros/jazzy/setup.bash`
- Source project setup: `source /sec_bot/install/setup.bash`

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