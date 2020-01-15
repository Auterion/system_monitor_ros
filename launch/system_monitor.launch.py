#!/usr/bin/env python3
"""
Launch system_monitor_node
"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='system_monitor_ros', node_executable='system_monitor_node', output='screen',
            parameters=["install/system_monitor_ros/share/system_monitor_ros/launch/monitor_parameters.yaml"]),
    ])
