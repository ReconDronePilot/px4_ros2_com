#!/usr/bin/env python

"""
Launch of Autonomous and joy controlled drone
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # Main control
    offboard_control = Node(
        package='px4_ros_com',
        executable='offboard_control',
        output='screen',
        shell=True
    )

    # To detect ps4 controller
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     #name='joy_node',
    #     output='screen',
    #     parameters=[{'deadzone': 0.2}]
    # )

    # # Joy To cmd_vel
    # controller = Node(
    #     package='joy_py',
    #     executable='ps4_control',
    #     output='screen'
    #     #parameters=[{'my_parameter': 'some_value'}]  # Add any additional parameters if needed
    # )

    return LaunchDescription([
        # joy_node,
        # controller,
        offboard_control
    ])
