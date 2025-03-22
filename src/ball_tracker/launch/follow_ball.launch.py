from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ball_tracker'),
                    'launch',
                    'orb_slam2_rgbd.launch.py'
                ])
            ])
        ),

        Node(
            package='ball_tracker',
            executable='image_publisher.py',
            name='image_publisher',
            parameters=[{'publish_rate': 30.0}]
        ),

        Node(
            package='ball_tracker',
            executable='sim_multi_ball_tracker.py',
            name='ball_tracker'
        ),

        Node(
            package='ball_tracker',
            executable='follow_ball',
            name='follow_ball',
            parameters=[{
                'base_speed': 0.2,
                'max_speed': 0.3,
                'angular_gain': 0.8,
                'stop_distance': 0.3,
                'search_speed': 0.5,
                'fov': 1.0,
                'ball_scale_factor': 0.05
            }]
        )
    ])