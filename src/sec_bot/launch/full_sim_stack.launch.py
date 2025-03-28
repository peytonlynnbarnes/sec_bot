from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Paths to launch files
    sec_bot_dir = get_package_share_directory('sec_bot')
    ball_tracker_dir = get_package_share_directory('ball_tracker')

    sim_launch = os.path.join(sec_bot_dir, 'launch', 'launch_sim.launch.py')
    cam_launch = os.path.join(ball_tracker_dir, 'launch', 'sim_cam.launch.py')
    follow_launch = os.path.join(ball_tracker_dir, 'launch', 'follow_ball.launch.py')

    purple_ball_path = os.path.join(sec_bot_dir, 'models', 'purple_ball.sdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Step 1: Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # Step 3: Launch simulated camera
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cam_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ]
        ),

        # Step 4: Launch follow ball logic
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(follow_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ]
        )
    ])
