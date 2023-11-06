from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server', default_value='DEBUG'),
        DeclareLaunchArgument('controller', default_value='DEBUG'),
        DeclareLaunchArgument('serial_communication', default_value='DEBUG'),
        DeclareLaunchArgument('visualizer', default_value='DEBUG'),

        Node(
            package='EXONET',
            executable='server',
            name='server',
            parameters=[{'logging__level': LaunchConfiguration('server')}]
        ),
        
        Node(
            package='EXONET',
            executable='controller',
            name='controller',
            parameters=[{'logging__level': LaunchConfiguration('controller')}]
        ),

        Node(
            package='EXONET',
            executable='serial_communication',
            name='serial_communication',
            parameters=[{'logging__level': LaunchConfiguration('serial_communication')}]
        ),

        Node(
            package='EXONET',
            executable='visualizer',
            name='visualizer',
            parameters=[{'logging__level': LaunchConfiguration('visualizer')}]
        )
    ])
