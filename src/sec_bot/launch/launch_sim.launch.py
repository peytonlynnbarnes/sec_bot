import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'sec_bot'

    world_file = 'empty.world'
    world_path = os.path.join(pkgs_share, 'worlds', world_file)
    gazebo_model_path = os.path.join(
        get_package_share_directory(package_name), 'models')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', gazebo_model_path)

    use_sim_time = LaunchConfiguration('use_sim_time')
    # use_ros2_control = LaunchConfiguration('use_ros2_control')
    # use_robot_localization = LaunchConfiguration('use_robot_localization')
    use_world_file = LaunchConfiguration('use_world_file')
    # use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_file = LaunchConfiguration('world_file')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items())

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        condition=IfCondition(LaunchConfiguration('use_world_file')),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't
    # really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')

    # Launch them all!
    return LaunchDescription([
        set_model_path,
        rsp,
        gazebo,
        spawn_entity,
    ])
