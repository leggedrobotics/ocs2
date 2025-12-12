import os

import launch
import launch_ros.actions

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='task_name',
            default_value='mpc'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_quadrotor_ros'), 'launch/visualize.launch.py')
            ),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        ),
        launch_ros.actions.Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_mpc',
            name='quadrotor_mpc',
            arguments=[LaunchConfiguration('task_name')],
            prefix= "",
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_dummy_test',
            name='quadrotor_dummy_test',
            arguments=[LaunchConfiguration('task_name')],
            prefix= "gnome-terminal --",
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_quadrotor_ros',
            executable='quadrotor_target',
            name='quadrotor_target',
            arguments=[LaunchConfiguration('task_name')],
            prefix= "gnome-terminal --",
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
