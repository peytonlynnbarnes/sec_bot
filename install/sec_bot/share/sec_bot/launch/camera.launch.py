from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_path',
            default_value='',
            description='Path to static image file'
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Camera index (0-based integer)'
        ),
        DeclareLaunchArgument(
            'resize_width',
            default_value='-1',
            description='Resize width (-1 for no resize)'
        ),

        Node(
            package='sec_bot_vision',
            executable='image_publisher',
            name='image_publisher',
            parameters=[{
                'publish_rate': 30.0,
                'image_path': LaunchConfiguration('image_path'),
                'camera_index': LaunchConfiguration('camera_index'),
                'resize_width': LaunchConfiguration('resize_width')
            }]
        ),
        Node(
            package='sec_bot_vision',
            executable='image_subscriber',
            name='image_subscriber'
        )
    ])

