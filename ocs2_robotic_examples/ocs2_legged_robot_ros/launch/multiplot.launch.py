import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='metrics_config',
            default_value=get_package_share_directory(
                'ocs2_legged_robot') + '/config/multiplot/zero_velocity.xml'
        ),
        launch_ros.actions.Node(
            package='rqt_multiplot',
            executable='rqt_multiplot',
            name='mpc_metrics',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_ros_interfaces'), 'launch/performance_indices.launch.py')
            ),
            launch_arguments={
                'mpc_policy_topic_name': 'legged_robot_mpc_policy'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
