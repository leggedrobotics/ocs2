import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_name'
        ),
        launch.actions.DeclareLaunchArgument(
            name='config_name',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_command',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value='ocs2_anymal_description'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_quadruped_interface'), 'launch/visualization.launch.py')
            ),
            launch_arguments={
                'description_name': LaunchConfiguration('description_name'),
            }.items()
        ),
        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration("description_name")],
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_mpc_node',
            name='ocs2_anymal_mpc_mpc_node',
            prefix="",
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_mpc',
            executable='ocs2_anymal_mpc_dummy_mrt_node',
            name='ocs2_anymal_mpc_dummy_mrt_node',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('description_name'), LaunchConfiguration('config_name')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_commands',
            executable='gait_command_node',
            name='gait_command_node',
            prefix="gnome-terminal --",
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_commands',
            executable='target_command_node',
            name='target_command_node',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('target_command')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_anymal_commands',
            executable='motion_command_node',
            name='motion_command_node',
            prefix="gnome-terminal --",
            arguments=['dummy'],
            output='screen'
        ),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
