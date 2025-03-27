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
    # Use the purple_ball.world file from the package's worlds folder.
    # If you want to use a Xacro version of the world file, rename it to have a .xacro extension.
    world_file = os.path.join(pkg_share, "worlds", "purple_ball.world")

    print(f"World file: {world_file}")
    
    # Check if the world file is a Xacro file by its extension; if so, process it.
    if world_file.endswith(".xacro"):
        doc = xacro.process_file(world_file)
        world_str = doc.toprettyxml(indent="  ")
        # Write the compiled world file to a temporary location.
        tmp_world_path = "/tmp/purple_ball.world"
        with open(tmp_world_path, "w") as f:
            f.write(world_str)
        world_file = tmp_world_path

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value=world_file, description="World file"
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
