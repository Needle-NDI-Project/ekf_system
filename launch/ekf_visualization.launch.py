from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ekf_system'),
            'config',
            'ekf_config.yaml'  # FIX: use ekf_config.yaml consistently
        ]),
        description='Path to the EKF configuration file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ekf_system'),
        'config',
        'ekf_visualization.rviz'
    ])

    ekf_node = Node(
        package='ekf_system',
        executable='ekf_node',
        name='ekf_node',
        parameters=[LaunchConfiguration('ekf_config')],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    ld = LaunchDescription()

    ld.add_action(ekf_config_arg)
    ld.add_action(use_rviz_arg)

    ld.add_action(ekf_node)
    ld.add_action(rviz_node)

    return ld
