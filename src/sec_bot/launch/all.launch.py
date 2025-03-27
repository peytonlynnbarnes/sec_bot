import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paths to the launch files
    sec_bot_launch_file = os.path.join(
        get_package_share_directory('sec_bot'),
        'launch',
        'launch_sim_model.launch.py'
    )
    ball_tracker_launch_file = os.path.join(
        get_package_share_directory('ball_tracker'),
        'launch',
        'sim_cam.launch.py'
    )
    follow_ball_launch_file = os.path.join(
        get_package_share_directory('ball_tracker'),
        'launch',
        'follow_ball.launch.py'
    )

    return launch.LaunchDescription([
        # Include sec_bot launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sec_bot_launch_file)
        ),
        # Include ball_tracker sim_cam launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ball_tracker_launch_file)
        ),
        # Execute ball_tracker sim_multi_ball_tracker node
        Node(
            package='ball_tracker',
            executable='sim_multi_ball_tracker',
            name='sim_multi_ball_tracker'
        ),
        # Include ball_tracker follow_ball launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_ball_launch_file)
        ),
    ])
