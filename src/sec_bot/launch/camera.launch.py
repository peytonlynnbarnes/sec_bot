import os  # temporary for file locations before turning launch scripts to seperate package

from launch import LaunchDescription
from launch import LaunchConfiguration
from launch_ros.actions import Node  # importing the nodes in this funny syntax


# generates the description of the launch file
def generate_launch_description():
    ld = LaunchDescription()  # calling itself

    pkgs_vision = os.path.join(get_package_share_directory("sec_bo"))

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )  # making the simulation track time

    # initializing the talker node
    vision_node = Node(
        package="sec_bot_vision",  # the package where everything is located in
        executable="talker",  # the name of the node proper
    )

    ld.add_action(vision_node)  # adding it to the list, LINKED LISTSSSSS

    return ld  # returning the description it found
