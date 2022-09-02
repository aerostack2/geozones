from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('config_file', default_value='config/geofences.json'),
        Node(
            package='geofencing',
            executable='geofencing_node',
            name='geofencing',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'config_file': LaunchConfiguration('config_file')}],
            output='screen',
            emulate_tty=True
        )
    ])
