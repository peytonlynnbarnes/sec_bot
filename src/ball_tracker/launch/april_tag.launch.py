from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    detector_node = Node(
        package='ball_tracker',
        executable='april_tag_detector',
        name='apriltag_detector',
        output='screen'
    )

    reporter_node = TimerAction(
        period=2.0,  # Delay to ensure detector is up first
        actions=[
            Node(
                package='ball_tracker',
                executable='april_tag_reporter',
                name='apriltag_reporter',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        detector_node,
        reporter_node
    ])