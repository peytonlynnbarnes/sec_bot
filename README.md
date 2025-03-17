# Usage 
### **Dependencies**
Make sure you have the following installed:
- **ROS 2** (Humble)
- **Python 3.8+**
- **OpenCV** 
- **cv_bridge** (ROS2 image conversion)
- **numpy**
### **Install & Build the Package**
```bash
cd ~/dev_ws/
git clone https://github.com/peytonlynnbarnes/sec_bot.git
colcon build --symlink-install
source install/setup.bash
```
### Launch Simulation
To run the simulation run the following 4 commands in separate terminals. (Don't forget to ```bash source install/setup.bash``` in all terminals)
```bash
ros2 launch sec_bot launch_sim.launch.py
# runs Gazebo simulation, robot state publisher, and spawn entity for ease in testing.
```
``` bash
ros2 launch ball_tracker sim_cam.launch.py
# launches image_subscriber that directly subscribes to /camera/image_raw  
```
```bash
ros2 run ball_tracker sim_multi_ball_tracker
# runs script that masks processed images and publishes /ball_position topic
```
``` bash
ros2 launch ball_tracker follow_ball.launch.py
# runs script that subscribes to /ball_position and publishes to /cmd_vel to follow ball
```

# sec_bot description  

This project is a robot created using ROS2 by the IEEE SEC 2025 Team designed to use a camera and motor controllers to track and follow objects.


## sec_bot package

This package contains the robot description as well as related launch files.
### URDF Robot Description  
  
<ins> inertial_macros </ins>: This contains inertial macros for the robot_core including inertial_box, inertial_cylinder, and inertial_sphere.  
  
<ins> robot_core </ins>: This contains the simulated robot's parameters and structure.  
  
<ins> gazebo_control </ins>  This contains differential drive parameters for the robot to move in Gazebo. 
  
<ins> camera.xacro </ins>  This contains the camera specifications for the simulated camera in Gazebo. 

### launch files 
   
<ins> rsp.launch.py </ins>: Launches robot state publisher 
  
<ins> launch_sim.launch.py  </ins>: Runs Gazebo simulation, robot state publisher, and spawn entity for ease in testing.

## ball_tracker package  
This package contains the ball tracking and following files as well as related launch files.  

### executables
<ins> image_publisher </ins>: publishes images from camera to ros topic.
<ins> image_subscriber </ins>: subscribes to topic to receive and process images.
  
<ins> multi_ball_tracker </ins>: uses OpenCV to mask processed images and publish coordinates of purple objects.  
Publishes to /ball_position topic and subscribes to camera/image_raw topic.  
  
<ins> sim_multi_ball_tracker </ins>: uses OpenCV to mask processed images and publish coordinates of purple objects.  
Publishes to /ball_position topic and subscribes to simulated camera/image_raw topic.  
  
<ins> follow_ball </ins>: receives ball position from /ball_position topic and publishes robot liner and angular speed to /cmd_vel to follow ball.   

### launch files
  
<ins> camera.launch.py </ins>: launches image_publisher and image_subscriber nodes
  
<ins> sim_cam.launch.py </ins> launches image_subscriber that directly subscribes to /camera/image_raw  
  
<ins> follow_ball.launch.py </ins> launches follow_ball script  

