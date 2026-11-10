"""
Goal-follower demo: opens turtlesim and the Turtle Pad side by side.

    ros2 launch milestone2_turtle_pad turtle_pad.launch.py
    ros2 launch milestone2_turtle_pad turtle_pad.launch.py typing:=true   # also open a terminal to type goals

Starts:
  turtlesim_node  - the simulator window
  goal_follower   - drives the turtle to each goal (P controller)
  goal_pad        - the clickable pad that sends goals
  goal_input      - (only with typing:=true) type "x y" goals in a new terminal
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Shell command that loads ROS + this package, used inside the new terminal.
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    pkg_setup = os.path.join(get_package_prefix('milestone2_turtle_pad'),
                             'share', 'milestone2_turtle_pad', 'local_setup.bash')
    source_cmd = f'source /opt/ros/{ros_distro}/setup.bash && source {pkg_setup}'

    return LaunchDescription([
        DeclareLaunchArgument('typing', default_value='false',
                              description='Also open a terminal for typing goals'),

        # The simulator. QT_QPA_PLATFORM=xcb makes it an X11 window, so the pad
        # can find where it is on screen (Wayland hides window positions).
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim',
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),

        # Drives the turtle to the latest goal on /goal_point.
        Node(package='milestone2_turtle_pad', executable='goal_follower', output='screen'),

        # The pad: click / follow-cursor, opens next to turtlesim.
        Node(package='milestone2_turtle_pad', executable='goal_pad', output='screen'),

        # Optional: a separate terminal window for typing goals.
        ExecuteProcess(
            cmd=['gnome-terminal', '--title=Type a goal', '--', 'bash', '-c',
                 f'{source_cmd} && ros2 run milestone2_turtle_pad goal_input'],
            condition=IfCondition(LaunchConfiguration('typing'))),
    ])
