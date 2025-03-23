from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time if true",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ball_tracker"),
                                "launch",
                                "orb_slam3_stereo.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            Node(
                package="ball_tracker",
                executable="image_publisher.py",
                name="image_publisher",
                parameters=[{"publish_rate": 30.0, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="ball_tracker",
                executable="sim_multi_ball_tracker.py",
                name="ball_tracker",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="ball_tracker",
                executable="follow_ball",
                name="follow_ball",
                parameters=[
                    {
                        "base_speed": 0.2,
                        "max_speed": 0.3,
                        "angular_gain": 0.8,
                        "stop_distance": 0.3,
                        "search_speed": 0.5,
                        "fov": 1.0,
                        "map_resolution": 0.05,
                        "ball_scale_factor": 0.05,
                        "stereo_baseline": 0.12,
                        "focal_length": 525.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
