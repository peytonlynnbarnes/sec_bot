from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    sec_bot_launch = os.path.join(
        get_package_share_directory('sec_bot'),
        'launch',
        'launch_sim_model.launch.py'
    )

    sim_cam_launch = os.path.join(
        get_package_share_directory('ball_tracker'),
        'launch',
        'sim_cam.launch.py'
    )

    follow_ball_launch = os.path.join(
        get_package_share_directory('ball_tracker'),
        'launch',
        'follow_ball.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # 1. Launch simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sec_bot_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 2. Launch simulated camera after delay
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sim_cam_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ]
        ),

        # 3. Launch ball tracker node
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='ball_tracker',
                    executable='sim_multi_ball_tracker',
                    name='sim_multi_ball_tracker',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),

        # 4. Launch ball follower logic
        TimerAction(
            period=9.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(follow_ball_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ]
        ),
    ])
