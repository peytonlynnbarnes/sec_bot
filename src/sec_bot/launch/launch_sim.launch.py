from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_robot_description(context, *args, **kwargs):
    pkg_share = get_package_share_directory("sec_bot")
    xacro_file = os.path.join(pkg_share, "description", "robot.urdf.xacro")

    # Process xacro to urdf
    doc = xacro.process_file(xacro_file)
    urdf_str = doc.toprettyxml()

    # Write to temp file
    urdf_path = "/tmp/robot.urdf"
    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-file", urdf_path, "-name", "sec_bot"],
            output="screen",
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("sec_bot")
    world_path = os.path.join(pkg_share, "worlds", "your_world.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value=world_path, description="World file"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    )
                ),
                launch_arguments={"world": LaunchConfiguration("world")}.items(),
            ),
            # Process the xacro and spawn robot
            OpaqueFunction(function=generate_robot_description),
        ]
    )
