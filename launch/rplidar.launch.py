#!/usr/bin/env python3
"""
==============================================================================
Launch file cho RPLidar (SLAMTEC) qua USB Serial
Giao thức: USB UART
Topic phát ra: /scan (LaserScan)
==============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================
    # CHỌN MODEL RPLIDAR (mỗi model có baud rate khác nhau)
    # ============================================
    # A1, A2M8: 115200 baud
    # A2M7, A2M12, A3, S1: 256000 baud
    # S2, S3, S2E: 1000000 baud
    # C1: 460800 baud
    # T1: UDP network (không phải serial)

    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[
                # Cổng serial
                {'serial_port': '/dev/ttyUSB0'},
                # Baud rate (chọn theo model thực tế)
                {'serial_baudrate': 115200},  # ← Thay đổi theo model!
                # Frame ID cho TF
                {'frame_id': 'laser_frame'},
                # Thời gian chờ
                {'timeout': 3.0},
            ],
        ),
    ])
