import os
import sys

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xml_path = get_package_share_directory('ocs2_ros_interfaces') + "/multiplot/performance_indices.xml"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='mpc_policy_topic_name',
            default_value='mpc_policy_topic_name'
        ),
        launch_ros.actions.Node(
            package='ocs2_ros_interfaces',
            executable='multiplot_remap',
            name='multiplot_remap',
            arguments=[LaunchConfiguration('mpc_policy_topic_name')], 
            output='screen'
        )
        # ,
        # launch_ros.actions.Node(
        #     package='rqt_multiplot',
        #     executable='rqt_multiplot',
        #     name='multiplot_performance_indices',
        #     prefix='--multiplot-run-all --multiplot-config' + xml_path,
        #     output='screen'
        # )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()