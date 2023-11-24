from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server_log_level', default_value='DEBUG'),
        DeclareLaunchArgument('controller_log_level', default_value='DEBUG'),
        DeclareLaunchArgument('serial_communication_log_level', default_value='DEBUG'),
        DeclareLaunchArgument('visualizer_log_level', default_value='DEBUG'),

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
            executable='serial_communicator',
            name='serial_communicator',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('serial_communication_log_level')}}]
        ),

        Node(
            package='EXONET',
            executable='gui',
            name='gui',
            parameters=[{'ros__parameters': {'ros__logging__logger_level': LaunchConfiguration('visualizer_log_level')}}]
        )
    ])
