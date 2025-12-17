#!/usr/bin/env python3

"""Launch file for MABI mobile manipulator."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    ocs2_mobile_manipulator_ros_dir = get_package_share_directory('ocs2_mobile_manipulator_ros')

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable rviz visualization'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Set nodes on debug mode'
    )

    # Robot-specific file paths
    urdf_file_arg = DeclareLaunchArgument(
        'urdfFile',
        default_value=PathJoinSubstitution([
            FindPackageShare('ocs2_robotic_assets'),
            'resources', 'mobile_manipulator', 'mabi_mobile', 'urdf', 'mabi_mobile.urdf'
        ]),
        description='The URDF model of the robot'
    )

    task_file_arg = DeclareLaunchArgument(
        'taskFile',
        default_value=PathJoinSubstitution([
            FindPackageShare('ocs2_mobile_manipulator'),
            'config', 'mabi_mobile', 'task.info'
        ]),
        description='The task file for the mpc'
    )

    lib_folder_arg = DeclareLaunchArgument(
        'libFolder',
        default_value=PathJoinSubstitution([
            FindPackageShare('ocs2_mobile_manipulator'),
            'auto_generated', 'mabi_mobile'
        ]),
        description='The library folder to generate CppAD codegen into'
    )

    # Include main mobile manipulator launch file
    mobile_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ocs2_mobile_manipulator_ros_dir, 'launch', 'mobile_manipulator.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'debug': LaunchConfiguration('debug'),
            'urdfFile': LaunchConfiguration('urdfFile'),
            'taskFile': LaunchConfiguration('taskFile'),
            'libFolder': LaunchConfiguration('libFolder'),
        }.items()
    )

    return LaunchDescription([
        rviz_arg,
        debug_arg,
        urdf_file_arg,
        task_file_arg,
        lib_folder_arg,
        mobile_manipulator_launch
    ])
