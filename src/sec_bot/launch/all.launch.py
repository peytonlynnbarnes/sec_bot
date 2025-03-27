import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory of the sec_bot package
    package_share_directory = get_package_share_directory('sec_bot')
    launch_dir = os.path.join(package_share_directory, 'launch')

    # Define each launch file to include
    ball_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ball_tracker_launch.py'))
    )
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch_sim.launch.py'))
    )
    launch_sim_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch_sim_model.launch.py'))
    )
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rsp.launch.py'))
    )

    # Create the overall launch description
    ld = LaunchDescription()

    # Add each included launch file to the description
    ld.add_action(ball_tracker_launch)
    ld.add_action(launch_sim)
    ld.add_action(launch_sim_model)
    ld.add_action(rsp_launch)

    return ld
