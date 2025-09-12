from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'CircularFormation_CoupledOscillators'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy', 
    ],
    zip_safe=True,
    maintainer='monkorusan',
    maintainer_email='monkorusan@example.com',
    description='Circular formation control in ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'central = CircularFormation_CoupledOscillators.central:main',
            'marker_publisher = CircularFormation_CoupledOscillators.marker_publisher:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
)
