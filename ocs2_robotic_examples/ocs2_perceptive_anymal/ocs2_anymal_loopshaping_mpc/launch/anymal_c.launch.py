import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    target_command = get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    description_name = get_package_share_directory('ocs2_anymal_models') + "/urdf/anymal_camel_rsl.urdf"
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_anymal_loopshaping_mpc'), 'launch/mpc.launch.py')
            ),
            launch_arguments={
                'robot_name': 'anymal_c',
                'config_name': 'c_series',
                'description_name': description_name,
                'target_command': target_command
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
