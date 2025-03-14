# Installation Steps
To install and run the following ros2 code for the IEEE SEC Competition 2025 robot please do the following steps:
1. Install ros2-humble (you need Ubuntu 22.04 installed) via the following commands, in order:
    ```
    sudo apt install software-properties-common && sudo add-apt-repository universe
    ```
    ```
    sudo apt update && sudo apt install curl python3-pip -y
    ```
    ```
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    ```
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```
    ```
    sudo apt update && sudo apt upgrade
    ```
    ```
    sudo apt install ros-humble-desktop ros-dev-tools ros-humble-xacro ros-humble-gazebo-ros-pkgs git
    ```

2. Install opencv via pip (ew)
    ```
    pip install --upgrade pip
    ```
    ```
    pip install opencv-python-headless
    ```

2. Git clone the repo with the following command:
    ```
    git clone https://github.com/peytonlynnbarnes/sec_bot
    ```

3. CD into the sec\_bot directory via:  
    ```
    cd sec_bot
    ```
4. Build the entire repo using colcon:
    ```
    colcon build --symlink-install
    ```
5. Source both ros commands and the local ones:
    ```
    source /opt/ros/humble/setup.bash && source install/local_setup.bash
    ```
6. Launch the simulation (to ensure that entire package was installed correctly) via: 
    ```
    haha
    ```

# Usage Steps (on for example a rasberry pi)
Not yet built/work in progress
