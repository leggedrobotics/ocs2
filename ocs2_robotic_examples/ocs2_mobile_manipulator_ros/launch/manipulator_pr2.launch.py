#!/usr/bin/env python3

"""Launch file for PR2 mobile manipulator."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    ocs2_mobile_manipulator_ros_dir = get_package_share_directory('ocs2_mobile_manipulator_ros')

    # Get file paths
    urdf_file = os.path.join(
        ocs2_mobile_manipulator_ros_dir,
        'urdf',
        'pr2_mobile_manipulator.urdf'
    )

    task_file = os.path.join(
        ocs2_mobile_manipulator_ros_dir,
        'config',
        'mpc',
        'task.info'
    )

    lib_folder = '/tmp/ocs2'

    # Include main mobile manipulator launch file
    mobile_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ocs2_mobile_manipulator_ros_dir, 'launch', 'mobile_manipulator.launch.py')
        ),
        launch_arguments={
            'urdfFile': urdf_file,
            'taskFile': task_file,
            'libFolder': lib_folder,
        }.items()
    )

    return LaunchDescription([
        mobile_manipulator_launch
    ])
