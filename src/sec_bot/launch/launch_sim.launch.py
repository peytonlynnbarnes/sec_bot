import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "sec_bot"

    # Define paths
    world_file = "empty.world"
    world_path = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file  # fixed syntax
    )
    gazebo_model_path = os.path.join(
        get_package_share_directory(package_name), "models"
    )

    # set gazebo model path
    set_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)

    # include robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # launch gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"world": world_path}.items(),  # critical fix here
    )

    # spawn the robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "sec_bot"],
        output="screen",
    )

    return LaunchDescription(
        [
            set_model_path,
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
