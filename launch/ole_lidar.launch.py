#!/usr/bin/env python3
"""
==============================================================================
Launch file cho OLE Oleros2 LiDAR
Kết nối: Ethernet UDP (qua switch)
Topic phát ra: /scan (LaserScan), /laser_data_frame (PointCloud2 nếu có)
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ole_pkg_dir = get_package_share_directory('ros2_lidar')
    ole_launch_file = os.path.join(ole_pkg_dir, 'launch', 'ole2dv2_launch.py')

    ole_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ole_launch_file)
    )

    return LaunchDescription([
        ole_lidar_launch
    ])

