from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_tracker',
            executable='image_subscriber',  
            name='image_subscriber_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw'  
            }],
        ),
    ])
