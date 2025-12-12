import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_anymal_models') + "/config/visualize_urdf.rviz"
    # 'anymal_c':  
    # get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
    # 'camel':  
    # get_package_share_directory('ocs2_anymal_models') + "/urdf/anymal_camel_rsl.urdf"
    
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='robot_name',  
            default_value='anymal_c',  
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory('ocs2_robotic_assets') + "/resources/anymal_c/urdf/anymal.urdf"
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'publish_frequency': 100.0
                },
                {
                    'use_tf_static': True
                }
            ],
            arguments=[launch.substitutions.LaunchConfiguration("urdfFile")],
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_gui': True
                },
                {
                    'rate': 100.0
                }
            ]
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
