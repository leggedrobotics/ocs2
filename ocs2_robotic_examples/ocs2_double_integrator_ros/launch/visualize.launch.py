import os
import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_file = get_package_share_directory('ocs2_double_integrator_ros') + "/rviz/double_integrator.rviz"
    urdf_dir = get_package_share_directory("ocs2_robotic_assets")
    urdf_model_path = os.path.join(urdf_dir, "resources/double_integrator/urdf", "double_integrator.urdf")

    use_joint_state_publisher_argument = launch.actions.DeclareLaunchArgument(
        name='use_joint_state_publisher',
        default_value='true'
    )
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        arguments=[urdf_model_path],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        arguments=[urdf_model_path],
        condition=IfCondition(use_joint_state_publisher),
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='double_integrator',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    ld = launch.LaunchDescription([
        use_joint_state_publisher_argument,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
