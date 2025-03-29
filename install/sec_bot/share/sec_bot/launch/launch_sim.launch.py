import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import math

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='sec_bot' #<--- CHANGE ME
    
    # Get the launch directory
    pkg_dir = get_package_share_directory(package_name)
    
    # Path to the mining field model SDF file - using absolute path to avoid any path resolution issues
    world_file_path = os.path.join(pkg_dir, 'worlds', 'field.world')
    
    # Path to the boxes SDF file
    boxes_file_path = os.path.join(pkg_dir, 'models', 'boxes.sdf')
    
    # Confirm files exist and are readable
    if not os.path.isfile(world_file_path):
        raise FileNotFoundError(f"Could not find world file at {world_file_path}")
        
    if not os.path.isfile(boxes_file_path):
        raise FileNotFoundError(f"Could not find boxes file at {boxes_file_path}")
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_dir, 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    # Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'world': world_file_path,
                    'verbose': 'true'
                }.items()
             )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   # Position robot at the center right section of the field
                                   '-x', '0.388112',
                                   '-y', '0.450398',
                                   '-z', '0.1'],
                        output='screen')
    # Wait for a few seconds to ensure Gazebo is fully loaded
    delay_after_gazebo = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )
    
    # Spawn the first box in the upper section based on the drawing (16" section)
    # Converting inches to meters: 16" = 0.4064m from origin
    spawn_box1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-file', boxes_file_path,
                                 '-entity', 'box1',
                                 '-x', '-0.143764',    # 16 inches from left side
                                 '-y', '0.487172',    # 14.5 inches from center
                                 '-z', '0.1'],
                
                      output='screen')
                      
    # Add a small delay between spawning the boxes
    delay_between_boxes = ExecuteProcess(
        cmd=['sleep', '2'],
        output='screen'
    )
    
    # Spawn the second box in the lower section based on the drawing
    spawn_box2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-file', boxes_file_path,
                                 '-entity', 'box2',
                                 '-x', '0.4064',    # 16 inches from left side
                                 '-y', '-0.4871723',   # -14.5 inches from center
                                 '-z', '0.1'],
                      output='screen')
    # Launch them all! 
    # The order ensures that Gazebo is started first, then we wait, 
    # then spawn the robot and boxes in sequence
    return LaunchDescription([
        rsp,
        gazebo,
        delay_after_gazebo,  # Wait for Gazebo to start completely
        spawn_entity,        # Spawn the robot
        spawn_box1,          # Spawn the first box
        delay_between_boxes, # Add delay between box spawns
        spawn_box2           # Spawn the second box
    ])