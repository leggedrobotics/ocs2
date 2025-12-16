import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_quadruped_interface') + "/config/config.rviz"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='description_name',
            default_value=''
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=["-d", rviz_config_file]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
