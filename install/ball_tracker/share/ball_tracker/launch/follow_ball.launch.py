import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ball_tracker',
            executable='follow_ball',
            name='follow_ball',
            parameters=[{'use_sim_time': True}]
        )
    ])
