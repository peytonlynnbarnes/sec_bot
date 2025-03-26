# Usage 
### **Dependencies**
Make sure you have the following installed:
- **Ubuntu 24.04**
- **ROS 2** (Jazzy)
- **Python 3.8+**
- **OpenCV** 
- **cv_bridge** (ROS2 image conversion)
- **numpy**


To install all of the dependencies use the following bash script by copying this into your terminal:
```bash
# installing ros2 dependencies (making sure they are installed)
sudo apt install software-properties-common -y
sudo add-apt-repository universe 
sudo apt update -y
sudo apt install curl -y
# installing ros2 jazzy
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop ros-dev-tools ros-jazzy-xacro -y
# installing opencv for cameras
sudo apt-get install python3-opencv -y
sudo apt-get update -y
# installing gazebo dependancies
sudo apt-get install curl lsb-release gnupg -y
# installing gazebo for simulation
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -y
sudo apt-get install gz-harmonic -y
sudo apt-get install ros-jazzy-ros-gz -y
# installing pangolin for orb slam3
sudo apt update -y
sudo apt install -y build-essential cmake libeigen3-dev libglew-dev libpython3-dev python3-numpy ffmpeg libavcodec-dev libavformat-dev libswscale-dev
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
# installing the rest of orb slam 3 dependencies
sudo apt install pipx
pipx install wheel
pipx ensurepath
cmake ..
make -j$(nproc)
sudo make install
sudo ln -sf /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.6.0 /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5d
```

### ** (old) Install & Build the Package**
```bash
git clone https://github.com/peytonlynnbarnes/sec_bot.git --branch jazzy-stuff
cd sec_bot/src
```
Follow install instructions for: https://github.com/Mechazo11/ros2_orb_slam3
Make sure folder ros2_orb_slam3 is in src folder. 
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash 
```
### (old) Launch Simulation
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
