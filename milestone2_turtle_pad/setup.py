import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'milestone2_turtle_pad'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files so `ros2 launch` can find them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayushi-n',
    maintainer_email='228990708+ayushi-n@users.noreply.github.com',
    description='Turtle Pad: drive the turtlesim turtle to clicked points or follow the mouse with a P controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_follower = milestone2_turtle_pad.goal_follower:main',
            'goal_pad = milestone2_turtle_pad.goal_pad:main',
            'goal_input = milestone2_turtle_pad.goal_input:main',
        ],
    },
)
