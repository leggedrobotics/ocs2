import os
from launch.substitutions import LaunchConfiguration

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_task_file = get_package_share_directory('ocs2_mobile_manipulator') + '/config/franka/task.info'
    default_urdf_file = get_package_share_directory('ocs2_robotic_assets') + '/resources/mobile_manipulator/franka/urdf/panda.urdf'
    default_lib_folder = '/tmp/ocs2_mobile_manipulator_auto_generated'

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gdb_prefix',
            default_value='gdb -ex run --args'
        ),
        launch.actions.DeclareLaunchArgument(
            name='terminal_prefix',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=default_task_file
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=default_urdf_file
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=default_lib_folder
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix=LaunchConfiguration('gdb_prefix'),
            condition=launch.conditions.IfCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix=LaunchConfiguration('terminal_prefix'),
            condition=launch.conditions.UnlessCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            name='mobile_manipulator_dummy_mrt_node',
            prefix=LaunchConfiguration('terminal_prefix'),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            prefix=LaunchConfiguration('terminal_prefix'),
            output='screen',
            parameters=[]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
