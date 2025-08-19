from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('pidcontroll_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pidcontroll_pkg',
            executable='pidcontroll_node',
            name='pidcontroll_node',
            output='screen',
            parameters=[config_dir]
        )
    ])
