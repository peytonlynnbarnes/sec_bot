from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    vocab_path = PathJoinSubstitution([
        FindPackageShare("ball_tracker"),
        "config/Vocabulary/ORBvoc.bin"
    ])

    settings_path = PathJoinSubstitution([
        FindPackageShare("ball_tracker"),
        "config/stereo.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time if true",
        ),
        Node(
            package="ros2_orb_slam3",
            executable="stereo_node_cpp",
            name="orb_slam3_stereo",
            output="screen",
            emulate_tty=True,
            arguments=[
                vocab_path,
                settings_path,
                "stereo"
            ],
            remappings=[
                ('/camera1/image_raw', '/camera/left/image_raw'),
                ('/camera2/image_raw', '/camera/right/image_raw')
            ]
        )
    ])
