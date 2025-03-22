# Usage 
### **Dependencies**
Make sure you have the following installed:
- **Ubuntu 24.04**
- **ROS 2** (Jazzy)
- **Python 3.8+**
- **OpenCV** 
- **cv_bridge** (ROS2 image conversion)
- **numpy**

To install all of the dependencies use the following script by copying this:
```
sudo apt install software-properties-common -y &&
sudo add-apt-repository universe &&
sudo apt update -y &&
sudo apt install curl python3-pip -y &&
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&
sudo apt update && 
sudo apt upgrade &&
sudo apt install ros-jazzy-desktop ros-dev-tools ros-jazzy-xacro ros-jazzy-gazebo-ros-pkgs -y &&
pip install --upgrade pip &&
pip install opencv-python-headless
```

### **Install & Build the Package**
```bash
git clone https://github.com/peytonlynnbarnes/sec_bot.git
cd sec_bot/src/ball_tracker/config/Vocabulary
wget https://github.com/raulmur/ORB_SLAM2/raw/refs/heads/master/Vocabulary/ORBvoc.txt.tar.gz
tar xf ORBvoc.txt.tar.gz
rm -rf ORBvoc.txt.tar.gz
cd ../../../..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash # make sure to source /opt/run/ros/humble/setup.bash
```
### Launch Simulation
To run the simulation run the following 4 commands in separate terminals. (Don't forget to ```bash source install/setup.bash``` in all terminals)
```bash
# runs Gazebo simulation, robot state publisher, and spawn entity for ease in testing.
ros2 launch sec_bot launch_sim.launch.py
```
``` bash
# launches image_subscriber that directly subscribes to /camera/image_raw  
ros2 launch ball_tracker sim_cam.launch.py
```
```bash
# runs script that masks processed images and publishes /ball_position topic
ros2 run ball_tracker sim_multi_ball_tracker
```
``` bash
# runs script that subscribes to /ball_position and publishes to /cmd_vel to follow ball
ros2 launch ball_tracker follow_ball.launch.py
```

# sec_bot Description  

This project is a robot created using ROS2 by the IEEE SEC 2025 Hardware Competition Team designed to use a camera and motor controllers to track and follow purple dice on a field, as well as pick up 2 boxes to sort the dice into.


## sec_bot package

This package contains the robot description as well as related launch files.
### URDF Robot Description  
  
<ins> inertial_macros </ins>: This contains inertial macros for the robot_core including inertial_box, inertial_cylinder, and inertial_sphere.  
  
<ins> robot_core </ins>: This contains the simulated robot's parameters and structure.  
  
<ins> gazebo_control </ins>  This contains differential drive parameters for the robot to move in Gazebo. 
  
<ins> camera.xacro </ins>  This contains the camera specifications for the simulated camera in Gazebo. 

### Launch Files 
   
<ins> rsp.launch.py </ins>: Launches robot state publisher 
  
<ins> launch_sim.launch.py  </ins>: Runs Gazebo simulation, robot state publisher, and spawn entity for ease in testing.

## ball_tracker package 
This package contains the ball tracking and following files as well as related launch files.  

### Executables
<ins> image_publisher </ins>: publishes images from camera to ros topic.
<ins> image_subscriber </ins>: subscribes to topic to receive and process images.
  
<ins> multi_ball_tracker </ins>: uses OpenCV to mask processed images and publish coordinates of purple objects. Publishes to /ball_position topic and subscribes to camera/image_raw topic.  
  
<ins> sim_multi_ball_tracker </ins>: uses OpenCV to mask processed images and publish coordinates of purple objects. Publishes to /ball_position topic and subscribes to simulated camera/image_raw topic.  
  
<ins> follow_ball </ins>: receives ball position from /ball_position topic and publishes robot liner and angular speed to /cmd_vel to follow ball.   

### Launch Files
  
<ins> camera.launch.py </ins>: launches image_publisher and image_subscriber nodes
  
<ins> sim_cam.launch.py </ins>: launches image_subscriber that directly subscribes to /camera/image_raw  
  
<ins> follow_ball.launch.py: </ins> launches follow_ball script
