#!/usr/bin/env python3
"""
Launch system_monitor_node
"""

import os
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    parameters_file_path = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        launch_ros.actions.Node(
            package='system_monitor_ros', node_executable='system_monitor_node', output='screen',
            parameters=[parameters_file_path + "/monitor_parameters.yaml"]),
    ])
