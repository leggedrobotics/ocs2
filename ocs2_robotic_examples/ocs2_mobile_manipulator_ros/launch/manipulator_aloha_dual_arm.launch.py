#!/usr/bin/env python3
"""Launch file for ALOHA dual arm mobile manipulator example."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directories
    ocs2_mobile_manipulator_ros_dir = get_package_share_directory('ocs2_mobile_manipulator_ros')
    ocs2_robotic_assets_dir = get_package_share_directory('ocs2_robotic_assets')
    ocs2_mobile_manipulator_dir = get_package_share_directory('ocs2_mobile_manipulator')

    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable rviz visualization'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Set nodes on debug mode'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdfFile',
        default_value=os.path.join(ocs2_robotic_assets_dir, 'resources', 'mobile_manipulator', 'aloha', 'urdf', 'vx300s.urdf'),
        description='The URDF model of the robot'
    )
    
    task_file_arg = DeclareLaunchArgument(
        'taskFile',
        default_value=os.path.join(ocs2_mobile_manipulator_dir, 'config', 'aloha', 'task.info'),
        description='The task file for the mpc'
    )
    
    lib_folder_arg = DeclareLaunchArgument(
        'libFolder',
        default_value=os.path.join(ocs2_mobile_manipulator_dir, 'auto_generated', 'aloha_dual_arm'),
        description='The library folder to generate CppAD codegen into'
    )

    nodes = []

    # left arm MPC node
    nodes.append(Node(
        package='ocs2_mobile_manipulator_ros',
        # executable='mobile_manipulator_dual_arm_mpc_node',
        executable='mobile_manipulator_mpc_node',
        name='left_arm_mpc_node',
        namespace='left_arm',
        parameters=[
            {'armSide': 'LEFT'},
            {'robotName': 'mobile_manipulator'},
            {'taskFile': LaunchConfiguration('taskFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'urdfFile': LaunchConfiguration('urdfFile')}
        ],
        output='screen'
    ))

    # right arm MPC node
    nodes.append(Node(
        package='ocs2_mobile_manipulator_ros',
        # executable='mobile_manipulator_dual_arm_mpc_node',
        executable='mobile_manipulator_mpc_node',
        name='right_arm_mpc_node',
        namespace='right_arm',
        parameters=[
            {'armSide': 'RIGHT'},
            {'robotName': 'mobile_manipulator'},
            {'taskFile': LaunchConfiguration('taskFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'urdfFile': LaunchConfiguration('urdfFile')}
        ],
        output='screen'
    ))

    # left arm MRT node
    nodes.append(Node(
        package='ocs2_mobile_manipulator_ros',
        # executable='mobile_manipulator_dual_arm_dummy_mrt_node',
        executable='mobile_manipulator_dummy_mrt_node',
        name='left_arm_mrt_node',
        namespace='left_arm',
        parameters=[
            {'armSide': 'LEFT'},
            {'robotName': 'mobile_manipulator'},
            {'taskFile': LaunchConfiguration('taskFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'urdfFile': LaunchConfiguration('urdfFile')}
        ],
        output='screen'
    ))

    # right arm MRT node
    nodes.append(Node(
        package='ocs2_mobile_manipulator_ros',
        # executable='mobile_manipulator_dual_arm_dummy_mrt_node',
        executable='mobile_manipulator_dummy_mrt_node',
        name='right_arm_mrt_node',
        namespace='right_arm',
        parameters=[
            {'armSide': 'RIGHT'},
            {'robotName': 'mobile_manipulator'},
            {'taskFile': LaunchConfiguration('taskFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'urdfFile': LaunchConfiguration('urdfFile')}
        ],
        output='screen'
    ))

    # # RViz node
    # nodes.append(Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(ocs2_mobile_manipulator_ros_dir, 'rviz', 'mobile_manipulator.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     output='screen'
    # ))

    return LaunchDescription([
        rviz_arg,
        debug_arg,
        urdf_file_arg,
        task_file_arg,
        lib_folder_arg,
    ] + nodes)
