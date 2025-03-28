from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # ORB-SLAM3 Node
        Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name='orb_slam3_stereo',
            output='screen',
            arguments=[
                '/microros_ws/install/ball_tracker/share/ball_tracker/config/Vocabulary/ORBvoc.bin',
                '/microros_ws/install/ball_tracker/share/ball_tracker/config/stereo.yaml',
                'stereo'
            ],
            remappings=[
                ('/camera1/image_raw', '/camera/left/image_raw'),
                ('/camera2/image_raw', '/camera/right/image_raw'),
                ('/orb_slam3/map', '/map')
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Ball tracker
        TimerAction(
            period=1.0,
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

        # Ball follower
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ball_tracker',
                    executable='follow_ball',
                    name='follow_ball',
                    output='screen',
                    parameters=[{
                        'base_speed': 0.2,
                        'max_speed': 0.3,
                        'angular_gain': 0.8,
                        'stop_distance': 0.3,
                        'search_speed': 0.5,
                        'fov': 1.0,
                        'map_resolution': 0.05,
                        'ball_scale_factor': 0.05,
                        'stereo_baseline': 0.12,
                        'focal_length': 525.0,
                        'use_sim_time': use_sim_time
                    }]
                )
            ]
        )
    ])
