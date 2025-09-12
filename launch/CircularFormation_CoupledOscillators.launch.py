from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('CircularFormation_CoupledOscillators'),
        'rviz',
        'CircularFormation_CoupledOscillators.rviz'
    )

    return LaunchDescription([
        Node(
            package='CircularFormation_CoupledOscillators',
            executable='central',
            name='central',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('CircularFormation_CoupledOscillators'), 'params', 'robot_params.yaml')]
        ),
        Node(
            package='CircularFormation_CoupledOscillators',
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