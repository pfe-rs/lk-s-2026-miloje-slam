from setuptools import find_packages, setup

package_name = 'my_robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='veljkogalovic',
    maintainer_email='veljkogalovic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'slam_processor = my_robot_slam.slam_processor:main',
            'mock_laser = my_robot_slam.mock_laser:main',
            'odom_publisher = my_robot_slam.odom_publisher:main',
            'battery_monitor = my_robot_slam.battery_monitor:main',
            'path_planner = my_robot_slam.path_planner:main',
            'motion_planner = my_robot_slam.motion_planner.main',
            'desni_motor = my_robot_slam.desni_motor:main',
            'levi_motor = my_robot_slam.levi_motor:main',
            'komunikacija = my_robot_slam.komunikacija:main'
        ],
    },
)
