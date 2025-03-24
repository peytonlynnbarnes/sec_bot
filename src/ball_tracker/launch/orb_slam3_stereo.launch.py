from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name='orb_slam3_stereo',    
            output='screen',
            parameters=[{
                'vocabulary_file_path': PathJoinSubstitution([
                    FindPackageShare('ball_tracker'),
                    'config/Vocabulary/ORBvoc.txt'
                ]),
                'settings_file_path': PathJoinSubstitution([
                    FindPackageShare('ball_tracker'),
                    'config/stereo.yaml'
                ]),
                'use_viewer': True,
                'use_sim_time': use_sim_time,
                'left_image_topic': '/camera/left/image_raw',
                'right_image_topic': '/camera/right/image_raw'
            }],
            remappings=[
                ('/orb_slam3/map', '/map')
            ]
        )
    ])