#!/usr/bin/env python3

"""Launch file for Kinova mobile manipulator."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directories
    ocs2_mobile_manipulator_ros_dir = get_package_share_directory('ocs2_mobile_manipulator_ros')
    ocs2_robotic_assets_dir = get_package_share_directory('ocs2_robotic_assets')
    ocs2_mobile_manipulator_dir = get_package_share_directory('ocs2_mobile_manipulator')

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

    urdf_file_arg = DeclareLaunchArgument(
        'urdfFile',
        default_value=os.path.join(ocs2_robotic_assets_dir, 'resources', 'mobile_manipulator', 'kinova', 'urdf', 'j2n6s300.urdf'),
        description='The URDF model of the robot'
    )

    task_file_arg = DeclareLaunchArgument(
        'taskFile',
        default_value=os.path.join(ocs2_mobile_manipulator_dir, 'config', 'kinova', 'task_j2n6.info'),
        description='The task file for the mpc'
    )

    lib_folder_arg = DeclareLaunchArgument(
        'libFolder',
        default_value=os.path.join(ocs2_mobile_manipulator_dir, 'auto_generated', 'kinova', 'j2n6'),
        description='The library folder to generate CppAD codegen into'
    )

    # Include main launch file
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