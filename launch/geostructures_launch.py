from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone_sim_0'),
        DeclareLaunchArgument(
            'config_file', default_value='geostructures/geofences.json'),
        DeclareLaunchArgument('mode', default_value='gps'),  # gps or cartesian
        Node(
            package='geostructures',
            executable='geostructures_node',
            name='geostructures',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'config_file': LaunchConfiguration(
                'config_file')}, {'mode': LaunchConfiguration('mode')}],
            output='screen',
            emulate_tty=True
        )
    ])
