"""
Shape-drawing demo (Milestone 1): opens turtlesim and starts the shape drawer.

    ros2 launch milestone1_shape_drawer shapes.launch.py                # draws a square
    ros2 launch milestone1_shape_drawer shapes.launch.py shape:=star    # pick a shape
    ros2 launch milestone1_shape_drawer shapes.launch.py shape:=circle size:=2.5

Shapes: square, triangle, pentagon, hexagon, star, circle.
While it runs you can switch shape from another terminal:
    ros2 param set /shape_drawer shape hexagon
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('shape', default_value='square',
                              description='square, triangle, pentagon, hexagon, star or circle'),
        DeclareLaunchArgument('size', default_value='3.0',
                              description='Radius of the shape (turtlesim window is 11 wide)'),

        # The simulator window. QT_QPA_PLATFORM=xcb runs it as an X11 window:
        # as a native Wayland window, turtlesim sometimes crashed on Ctrl+C.
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim',
             additional_env={'QT_QPA_PLATFORM': 'xcb'}),

        # Draws the chosen shape, then waits for a new shape request.
        Node(package='milestone1_shape_drawer', executable='shape_drawer', output='screen',
             parameters=[{
                 'shape': LaunchConfiguration('shape'),
                 # size:=3 would be read as a whole number; the node needs a
                 # decimal number, so convert it to float explicitly.
                 'size': ParameterValue(LaunchConfiguration('size'), value_type=float),
             }]),
    ])
