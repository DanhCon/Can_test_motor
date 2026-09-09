#!/usr/bin/env python3
"""
==============================================================================
[STANDALONE DEBUG ONLY] Launch file cho RPLidar (SLAMTEC) qua USB Serial
LƯU Ý: File này CHỈ DÙNG ĐỂ KIỂM TRA ĐỘC LẬP CẢM BIẾN RPLIDAR.
KHÔNG chạy file này cùng lúc với robot.launch.py (vì robot.launch.py đã tích hợp sẵn).
Chuỗi topic chuẩn: /scan_raw (thô từ driver) -> laser_filters -> /scan (đã gọt góc +-119 độ)
TF chuẩn: base_link -> laser_frame (x=0.20, y=0.0, z=0.15)
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    # Tham số cấu hình cổng kết nối
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Cổng serial kết nối RPLidar'
    )
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baud rate của RPLidar (A1/A2M8: 115200, A2M7/A3/S1: 256000, S2/S3: 1000000, C1: 460800)'
    )
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='TF frame ID của cảm biến LiDAR'
    )

    # 1. RPLIDAR Driver Node (phát thô vào /scan_raw)
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'timeout': 3.0,
        }],
        remappings=[('/scan', '/scan_raw')]
    )

    # 2. Bộ lọc tia laser_filters: /scan_raw -> /scan (gọt góc chắn sau lưng xe +-119 độ)
    angular_filter_config = os.path.join(pkg_dir, 'config', 'angular_filter.yaml')
    filter_laser_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[angular_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ]
    )

    # 3. Static TF Publisher: base_link -> laser_frame (Vị trí lắp LiDAR trên xe)
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
        declare_serial_port,
        declare_serial_baudrate,
        declare_frame_id,
        rplidar_node,
        filter_laser_node,
        static_tf_node,
    ])
