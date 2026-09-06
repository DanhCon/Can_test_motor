#!/usr/bin/env python3
"""
==============================================================================
Launch file tổng hợp cho LIDAR (OLE + RPLidar)
Kết hợp cả 2 loại LIDAR vào 1 launch file thuận tiện
==============================================================================

Cách dùng:
  ros2 launch can_test_motor lidar.launch.py

Tham số override:
  ros2 launch can_test_motor lidar.launch.py \
    use_ole:=true \
    use_rplidar:=false \
    ole_ip:=192.168.1.101
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')
    default_ole_config = os.path.join(pkg_dir, 'config', 'ole2dv2.yaml')

    use_ole = LaunchConfiguration('use_ole')
    use_rplidar = LaunchConfiguration('use_rplidar')
    ole_param_file = LaunchConfiguration('ole_param_file')

    # 1. Khai báo Launch Arguments
    declare_use_ole = DeclareLaunchArgument(
        'use_ole',
        default_value='true',
        description='Bật/tắt cảm biến OLE LiDAR qua mạng Ethernet (true/false)'
    )
    declare_use_rplidar = DeclareLaunchArgument(
        'use_rplidar',
        default_value='false',
        description='Bật/tắt cảm biến RPLidar qua cổng USB (true/false)'
    )
    declare_ole_config = DeclareLaunchArgument(
        'ole_param_file',
        default_value=default_ole_config,
        description='Đường dẫn file cấu hình YAML của OLE LiDAR'
    )

    # 2. Node OLE LiDAR (Ethernet UDP) qua package ros2_lidar
    ole_launch_file = os.path.join(get_package_share_directory('ros2_lidar'), 'launch', 'ole2dv2_launch.py')
    ole_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ole_launch_file),
        condition=IfCondition(use_ole)
    )

    # 3. Node RPLidar (USB Serial)
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
        }],
        condition=IfCondition(use_rplidar)
    )

    # 4. Static TF Publisher: base_link -> laser_frame (Vị trí lắp LiDAR trên xe)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=['--x', '0.20', '--y', '0.0', '--z', '0.15',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    return LaunchDescription([
        declare_use_ole,
        declare_use_rplidar,
        declare_ole_config,
        ole_node,
        rplidar_node,
        static_tf_node
    ])
