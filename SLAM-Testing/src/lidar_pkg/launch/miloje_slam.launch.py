import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Inside miloje_slam.launch.py, update the executables to match your entry_points:
        Node(
            package='lidar_pkg',
            executable='lidar_scan_node',  # <-- REMOVE '.py'
            name='lidar_scan_node',
            parameters=[{'port_name': '/dev/ttyUSB0'}],
            output='screen'
        ),
        Node(
            package='lidar_pkg',
            executable='motion_planner_node',  # <-- REMOVE '.py'
            name='motion_planner_node',
            output='screen'
        ),
        Node(
            package='lidar_pkg',
            executable='slam_mapping_node',  # <-- REMOVE '.py'
            name='slam_mapping_node',
            output='screen'
        ),
        Node(
            package='lidar_pkg',
            executable='path_planner_node',  # <-- REMOVE '.py'
            name='path_planner_node',
            output='screen'
        ),
        # 5. Kickstart Service Call (Triggers after a brief delay)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/lidar_scan_node/scan_request', 'std_srvs/srv/Trigger'],
            output='screen'
        )
    ])
