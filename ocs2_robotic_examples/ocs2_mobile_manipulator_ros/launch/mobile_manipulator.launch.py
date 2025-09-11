#!/usr/bin/env python3

"""Main launch file for mobile manipulator."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


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

    urdf_file_arg = DeclareLaunchArgument(
        'urdfFile',
        description='The URDF model of the robot'
    )

    task_file_arg = DeclareLaunchArgument(
        'taskFile',
        description='The task file for the mpc'
    )

    lib_folder_arg = DeclareLaunchArgument(
        'libFolder',
        description='The library folder to generate CppAD codegen into'
    )

    # Set parameters
    set_task_file_param = SetParameter(
        name='taskFile',
        value=LaunchConfiguration('taskFile')
    )

    set_urdf_file_param = SetParameter(
        name='urdfFile',
        value=LaunchConfiguration('urdfFile')
    )

    set_lib_folder_param = SetParameter(
        name='libFolder',
        value=LaunchConfiguration('libFolder')
    )

    # Include visualization launch file
    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ocs2_mobile_manipulator_ros_dir, 'launch', 'visualize.launch.py')
        ),
        launch_arguments={
            'urdfFile': LaunchConfiguration('urdfFile'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # MPC node
    mpc_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_mpc_node',
        name='mobile_manipulator_mpc_node',
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
        ]
    )

    # Dummy MRT node
    dummy_mrt_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_dummy_mrt_node',
        name='mobile_manipulator_dummy_mrt_node',
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
        ]
    )

    # Target node (only when rviz is enabled)
    target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_target',
        name='mobile_manipulator_target',
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        debug_arg,
        urdf_file_arg,
        task_file_arg,
        lib_folder_arg,
        set_task_file_param,
        set_urdf_file_param,
        set_lib_folder_param,
        visualize_launch,
        mpc_node,
        dummy_mrt_node,
        target_node
    ])
