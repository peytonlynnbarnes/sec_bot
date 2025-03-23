from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ball_tracker",
                executable="image_subscriber",  # Ensure your setup.py entry point is correct
                name="image_subscriber_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw"  # Use '/camera2/image_raw' if needed
                    }
                ],
            ),
        ]
    )
