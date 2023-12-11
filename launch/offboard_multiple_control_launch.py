#!/usr/bin/env python

"""
Launch of Autonomous and joy controlled drone
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Main control
    manual_control = Node(
        package='px4_ros_com',
        executable='manual_control',
        output='screen',
        shell=True,
    )

    # To detect ps4 controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1}],
    )

    # Joy To cmd_vel
    controller = Node(
        package='px4_ros_com',
        executable='ps4_control.py',
        output='screen',
        #parameters=[{'my_parameter': 'some_value'}]  # Add any additional parameters if needed
    ),

    return LaunchDescription([
        controller,
        manual_control
    ])
