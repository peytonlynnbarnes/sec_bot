from setuptools import setup
from glob import glob
import os

package_name = 'sec_bot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py')) or []),  # safe handling
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='your.email@example.com',
    description='SEC Bot vision system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = sec_bot_vision.image_publisher:main',
            'image_subscriber = sec_bot_vision.image_subscriber:main',
        ],
    },
)