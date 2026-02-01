from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('CircularFormation'),
        'rviz',
        'CircularFormation.rviz'
    )

    return LaunchDescription([
        Node(
            package='CircularFormation',
            executable='central',
            name='central',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('CircularFormation'), 'params', 'robot_params.yaml')]
        ),
        Node(
            package='CircularFormation',
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