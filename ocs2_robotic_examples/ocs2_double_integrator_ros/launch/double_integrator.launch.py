import os
import sys

import launch
import launch_ros.actions

from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_arg = launch.actions.DeclareLaunchArgument(
        name='rviz',
        default_value='true'
    )
    task_name_arg = launch.actions.DeclareLaunchArgument(
        name='task_name',
        default_value='mpc'
    )
    debug_arg = launch.actions.DeclareLaunchArgument(
        name='debug',
        default_value='false'
    )

    ld = launch.LaunchDescription([
        rviz_arg,
        task_name_arg,
        debug_arg
    ])

    # TODO rviz_condition=launch.conditions.IfCondition(LaunchConfiguration("rviz"))
    ld.add_action(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_double_integrator_ros'), 'launch/visualize.launch.py')
            ),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_mpc',
            name='double_integrator_mpc',
            arguments=[LaunchConfiguration('task_name')],
            prefix= "gnome-terminal -- gdb --args",
            condition=launch.conditions.IfCondition(LaunchConfiguration("debug")),
            output='screen'
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_mpc',
            name='double_integrator_mpc',
            arguments=[LaunchConfiguration('task_name')],
            prefix= "",
            condition=launch.conditions.UnlessCondition(LaunchConfiguration("debug")),
            output='screen'
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_dummy_test',
            name='double_integrator_dummy_test',
            arguments=[LaunchConfiguration('task_name')],
            prefix="gnome-terminal --",
            output='screen'
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='ocs2_double_integrator_ros',
            executable='double_integrator_target',
            name='double_integrator_target',
            arguments=[LaunchConfiguration('task_name')],
            prefix="gnome-terminal --",
            output='screen'
        )
    )
    return ld

if __name__ == '__main__':
    generate_launch_description()
