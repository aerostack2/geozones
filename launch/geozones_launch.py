from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone_0'),
        DeclareLaunchArgument(
            'config_file', default_value='geozones/geozones.json'),
        DeclareLaunchArgument(
            'debug_rviz', default_value='true', description='Enable Polygon topic publisher for RViz visualization'),
        Node(
            package='geozones',
            executable='geozones_node',
            name='geozones',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'config_file': LaunchConfiguration(
                'config_file')}, {'debug_rviz': LaunchConfiguration('debug_rviz')}],
            output='screen',
            emulate_tty=True
        )
    ])
