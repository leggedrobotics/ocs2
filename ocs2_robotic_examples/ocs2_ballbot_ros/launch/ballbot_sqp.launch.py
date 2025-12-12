import os
from launch.substitutions import LaunchConfiguration

import launch
import launch_ros.actions
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
                    'ocs2_ballbot_ros'), 'launch/visualize.launch.py')
            ),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        ),
        launch_ros.actions.Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_sqp',
            name='ballbot_sqp',
            prefix= "",
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_dummy_test',
            name='ballbot_dummy_test',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_target',
            name='ballbot_target',
            prefix="gnome-terminal --",
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
