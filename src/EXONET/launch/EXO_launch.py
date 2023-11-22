from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_log_level', default_value='INFO'),
        DeclareLaunchArgument('controller_log_level', default_value='INFO'),
        DeclareLaunchArgument('serial_communication_log_level', default_value='INFO'),
        DeclareLaunchArgument('visualizer_log_level', default_value='INFO'),

        Node(
            package='EXONET',
            executable='server',
            name='server',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('server_log_level')}}]
        ),
        
        Node(
            package='EXONET',
            executable='controller',
            name='controller',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('controller_log_level')}}]
        ),

        Node(
            package='EXONET',
            executable='serial_communication',
            name='serial_communication',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('serial_communication_log_level')}}]
        ),

        Node(
            package='EXONET',
            executable='gui',
            name='gui',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('visualizer_log_level')}}]
        )
    ])
