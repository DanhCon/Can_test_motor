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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')
    default_param_file = os.path.join(pkg_dir, 'config', 'ole2dv2.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Full path to the OLE LiDAR parameters YAML file'
    )

    # 1. Node OLE LiDAR Driver
    ole_node = Node(
        package='oleros2',
        executable='oleros2_node',
        name='ole_lidar',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
    )

    # 2. Static TF: base_link -> laser_frame (Vị trí lắp LiDAR trên xe)
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
        param_file_arg,
        ole_node,
        static_tf_node,
    ])
