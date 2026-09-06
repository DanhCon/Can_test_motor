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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================
    # CÁC THAM SỐ TOÀN CỤC
    # ============================================
    use_ole = LaunchConfiguration('use_ole')
    use_rplidar = LaunchConfiguration('use_rplidar')
    ole_ip = LaunchConfiguration('ole_ip')

    return LaunchDescription([
        # ============================================
        # THAM SỐ CẤU HÌNH
        # ============================================
        DeclareLaunchArgument('use_ole', default_value='true', description='Bật OLE LiDAR (true/false)'),
        DeclareLaunchArgument('use_rplidar', default_value='false', description='Bật RPLidar qua USB (true/false)'),
        DeclareLaunchArgument('ole_ip', default_value='192.168.1.101', description='IP của OLE LiDAR'),

        # ============================================
        # OLE LIDAR (Ethernet)
        # ============================================
        GroupAction([
            IfCondition(IfCondition.value_if_true(use_ole)),
            Node(
                package='oleros2',
                executable='oleros2_node',
                name='ole_lidar',
                output='screen',
                parameters=[
                    {'ip_address': ole_ip},
                    {'frame_id': 'laser_frame'},
                ],
            ),
        ]),

        # ============================================
        # RPLIDAR (USB Serial)
        # ============================================
        GroupAction([
            IfCondition(IfCondition.value_if_true(use_rplidar)),
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='rplidar_node',
                output='screen',
                parameters=[
                    {'serial_port': '/dev/ttyUSB0'},
                    {'serial_baudrate': 115200},  # ← Chọn đúng baud theo model
                    {'frame_id': 'laser_frame'},
                ],
            ),
        ]),
    ])
