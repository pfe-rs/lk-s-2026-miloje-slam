from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files if you have any
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include any config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'battery_monitor = my_robot_slam.battery_monitor:main',
            'path_planner = my_robot_slam.path_planner:main',
            'motion_planner = my_robot_slam.motion_planner:main',
            'komunikacija = my_robot_slam.komunikacija:main',
            #LidarRviz opcioni fajl
        ],
    },
)
