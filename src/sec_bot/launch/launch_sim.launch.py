import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "sec_bot"

    # declare launch arguments
    DeclareLaunchArgument(
        "wait_for_gazebo",
        default_value="true",
        description="Wait for Gazebo to initialize",
    )

    # world and model paths
    world_path = os.path.join(
        get_package_share_directory(package_name), "worlds", "empty.world"
    )
    gazebo_model_path = os.path.join(
        get_package_share_directory(package_name), "models"
    )

    # environment setup
    set_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)

    # robot state publisher
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

    # gazebo with ROS plugins
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
        launch_arguments={
            "verbose": "true",
            "extra_gazebo_args": "-s libgazebo_ros_factory.so",
        }.items(),
    )

    # delayed entity spawner
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "sec_bot"],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("wait_for_gazebo")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("wait_for_gazebo", default_value="true"),
            set_model_path,
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
