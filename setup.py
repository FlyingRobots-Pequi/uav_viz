import os
from glob import glob
from setuptools import setup

package_name = 'uav_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Team',
    maintainer_email='your-email@example.com',
    description='UAV Visualization and Monitoring System for Real-time Drone Operations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_visualizer = uav_viz.uav_visualizer:main',
            'status_monitor = uav_viz.status_monitor:main',
            'mission_monitor = uav_viz.mission_monitor:main',
        ],
    },
) 