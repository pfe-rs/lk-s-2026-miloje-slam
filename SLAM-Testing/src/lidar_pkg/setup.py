import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # THIS LINE RIGHT HERE INCLUDES YOUR LAUNCH FILES:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='veljkogalovic',
    maintainer_email='veljkogalovic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_scan_node = scripts.lidar_scan_node:main',
            'path_planner_node = scripts.path_planner_node:main',
            'motion_planner_node = scripts.motion_planner_node:main',
            'slam_mapping_node = scripts.slam_mapping_node:main',
            'vector_deducer_node = scripts.vector_deducer_node:main'
            #LidarRviz opcioni fajl
        ],
    },
)