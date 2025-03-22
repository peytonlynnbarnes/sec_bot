from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam2_ros',
            executable='stereo_node',
            name='orb_slam2',
            parameters=[{
                'vocabulary_path': PathJoinSubstitution([
                    FindPackageShare('ball_tracker'),
                    'config/Vocabulary/ORBvoc.txt'
                ]),
                'settings_path': PathJoinSubstitution([
                    FindPackageShare('ball_tracker'),
                    'config/Stereo.yaml'
                ]),
                'camera_topic_left': '/camera/left/image_raw',
                'camera_topic_right': '/camera/right/image_raw',
                'use_sim_time': True
            }],
            output='screen'
        )
    ])