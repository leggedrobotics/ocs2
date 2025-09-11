#!/usr/bin/env python3

"""Visualization launch file for mobile manipulator."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    ocs2_mobile_manipulator_ros_dir = get_package_share_directory('ocs2_mobile_manipulator_ros')

    # Launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdfFile',
        description='The URDF model of the robot'
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('urdfFile')}
        ]
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz node
    rviz_config = os.path.join(
        ocs2_mobile_manipulator_ros_dir,
        'rviz',
        'mobile_manipulator.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        urdf_file_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
