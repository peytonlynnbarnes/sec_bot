# SEC Bot ROS2 Jazzy Docker Setup

## Prerequisites
- Docker
- Docker Compose (optional)

## Building the Docker Image
```bash
docker build -t sec-bot:jazzy .
```

## Running the Container
### Interactive Mode
```bash
docker run -it --rm sec-bot:jazzy bash
```

### Running Specific ROS2 Launch Commands
```bash
# Gazebo Simulation
docker run -it --rm sec-bot:jazzy ros2 launch sec_bot launch_sim.launch.py

# Camera Subscriber
docker run -it --rm sec-bot:jazzy ros2 launch ball_tracker sim_cam.launch.py
```

## Development Workflow
1. Build the image
2. Mount your local source code for development
3. Run specific ROS2 commands as needed

## Troubleshooting
- Ensure you have the latest Docker installed
- Check that your host system meets ROS2 Jazzy requirements
- Verify graphics and device passthrough for simulation

## Notes
- This Dockerfile is designed for development and testing
- Performance may vary compared to native installation
```
