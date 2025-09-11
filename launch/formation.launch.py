from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('formation'),
        'rviz',
        'formation.rviz'
    )

    return LaunchDescription([
        Node(
            package='formation',
            executable='central',
            name='central',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('formation'), 'params', 'robot_params.yaml')]
        ),
        Node(
            package='formation',
            executable='marker_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])