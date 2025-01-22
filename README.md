# Installation Steps
To install and run the following ros2 code for the IEEE SEC Competition 2025 robot please do the following steps:
1. Install ros2-humble (you need Ubuntu 22.04 installed) via the following commands, in order:
    ```
    sudo apt install software-properties-common && sudo add-apt-repository universe
    ```
    ```
    sudo apt update && sudo apt install curl python3-pip python3-colcon-common-extensions -y
    ```
    ```
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    ```
    sudo apt update && sudo apt upgrade
    ```
    ```
    sudo apt install ros-humble-desktop ros-dev-tools ros-humble-xacro
    ```

2. Git clone the repo with the following command:
    ```
    git clone https://github.com/peytonlynnbarnes/sec_bot
    ```

3. CD into the sec\_bot directory via:  
    ```
    cd sec_bot
    ```
4. Source both ros commands and the local ones:
    ```
    source /opt/ros/humble/setup.bash && source install/local_setup.bash
    ```
5. Build the entire repo using colcon:
    ```
    colcon build --symlink-install
    ```
6. Launch the simulation (to ensure that entire package was installed correctly) via: 
    ```
    ros2 launch sec_bot launch_sim.launch.py
    ```

# Usage Steps (on for example a rasberry pi)
Not yet built/work in progress
