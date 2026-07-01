import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Lidar Driver Node
        Node(
            package='lidar_pkg',
            executable='lidar_scan_node.py',
            name='lidar_scan_node',
            parameters=[{'port_name': '/dev/ttyUSB0'}],
            output='screen'
        ),
        
        # 2. Arduino Motor Controller (Miloje)
        Node(
            package='lidar_pkg',
            executable='motion_planner_node.py',
            name='motion_planner_node',
            output='screen'
        ),
        
        # 3. SLAM Mapping Node
        Node(
            package='lidar_pkg',
            executable='slam_mapping_node.py',
            name='slam_mapping_node',
            output='screen'
        ),
        
        # 4. Frontier Path Planner Node
        Node(
            package='lidar_pkg',
            executable='path_planner_node.py',
            name='path_planner_node',
            output='screen'
        ),
        
        # 5. Kickstart Service Call (Triggers after a brief delay)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/lidar_scan_node/scan_request', 'std_srvs/srv/Trigger'],
            output='screen'
        )
    ])