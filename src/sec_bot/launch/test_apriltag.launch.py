import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package name
    package_name = 'sec_bot'
    
    # Get the package directory
    pkg_dir = get_package_share_directory(package_name)
    
    # Get the absolute path to the model directory
    model_path = os.path.join(pkg_dir, 'models')
    
    # Set the Gazebo model path environment variable
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[model_path, ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    # Wait to ensure Gazebo is ready
    delay = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    
    # Spawn the AprilTag
    spawn_apriltag = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'apriltag',
            '-model', 'apriltag',  # This uses the model name from Gazebo's model database
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        delay,
        spawn_apriltag
    ])