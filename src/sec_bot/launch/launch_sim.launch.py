import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='sec_bot' #<--- CHANGE ME
    
    # Get the launch directory
    pkg_dir = get_package_share_directory(package_name)
    
    # Path to the mining field model SDF file - using absolute path to avoid any path resolution issues
    world_file_path = os.path.join(pkg_dir, 'worlds', 'field.world')
    
    # Confirm file exists and is readable
    if not os.path.isfile(world_file_path):
        raise FileNotFoundError(f"Could not find world file at {mining_field_model_path}")
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_dir, 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch Gazebo with an empty world first
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'world': world_file_path,
                    'verbose': 'true'
                }.items()
             )
    
    # Import the SDF world using the gazebo_ros model importer
    import_world = ExecuteProcess(
        cmd=['bash', '-c', f'sleep 5 && gz service -s /world/default/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req "sdf_filename: \\"{world_file_path}\\" allow_renaming: true name: \\"mining_field\\""'],
        output='screen'
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        import_world,  # Import the world after Gazebo has started
        spawn_entity,
    ])