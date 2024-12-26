from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('ekf_system'),
            'config',
            'ekf_config.yaml'
        ]),
        description='Path to the EKF configuration file'
    )

    ndi_launch_arg = DeclareLaunchArgument(
        'ndi_launch',
        default_value=PathJoinSubstitution([
            FindPackageShare('ndi_bringup'),
            'launch',
            'ndi_system.launch.py'
        ]),
        description='Path to NDI system launch file'
    )

    ndi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([LaunchConfiguration('ndi_launch')]),
        launch_arguments={}.items()
    )

    ekf_node = Node(
        package='ekf_system',
        executable='ekf_node',
        name='ekf_node',
        parameters=[LaunchConfiguration('ekf_config')],
        output='screen',
        remappings=[
            ('/ndi_tracker/state', '/ndi_tracker/state'),
            ('/cmd_vel', '/cmd_vel'),
            ('/needle_planning/trajectory', '/needle_planning/trajectory'),
            ('/zaber_system/motor_commands', '/zaber_system/motor_commands'),
            ('/ekf_state_estimate', '/ekf_state_estimate')
        ]
    )

    ld = LaunchDescription()

    ld.add_action(ekf_config_arg)
    ld.add_action(ndi_launch_arg)
    ld.add_action(ndi_launch)
    ld.add_action(ekf_node)

    return ld
