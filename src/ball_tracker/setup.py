from setuptools import find_packages, setup

package_name = "ball_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/ball_tracker/launch", ["launch/camera.launch.py"]),
        ("share/ball_tracker/launch", ["launch/sim_cam.launch.py"]),
        ("share/ball_tracker/launch", ["launch/follow_ball.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="peyton",
    maintainer_email="peyton0330@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_publisher = ball_tracker.image_publisher:main",
            "image_subscriber = ball_tracker.image_subscriber:main",
            "multi_ball_tracker = ball_tracker.multi_ball_tracker:main",
            "sim_multi_ball_tracker = ball_tracker.sim_multi_ball_tracker:main",
            "follow_ball = ball_tracker.follow_ball:main",
        ],
    },
)
