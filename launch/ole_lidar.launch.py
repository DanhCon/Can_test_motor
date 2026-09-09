#!/usr/bin/env python3
"""
==============================================================================
[STANDALONE DEBUG ONLY] Launch file cho OLE LiDAR kết hợp laser_filters
LƯU Ý: File này CHỈ DÙNG ĐỂ KIỂM TRA ĐỘC LẬP CẢM BIẾN OLE LIDAR.
KHÔNG chạy file này cùng lúc với robot.launch.py (vì robot.launch.py đã tích hợp sẵn).
Kiến trúc chuẩn:
  - lidar_driver (ros2_lidar): Phát tia quét thô 360 độ ra topic /scan_raw
  - laser_filters: Nhận /scan_raw, gọt góc che sau lưng xe (+-119 độ),
                   xuất bản tia quét chuẩn ra /scan (15 Hz)
  - Static TF: base_link -> laser_frame
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')
    filter_config = os.path.join(pkg_dir, 'config', 'angular_filter.yaml')

    ole_pkg_dir = get_package_share_directory('ros2_lidar')
    ole_launch_file = os.path.join(ole_pkg_dir, 'launch', 'ole2dv2_launch.py')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    declare_lidar_x = DeclareLaunchArgument('lidar_x', default_value='0.20', description='LiDAR X offset (m)')
    declare_lidar_y = DeclareLaunchArgument('lidar_y', default_value='0.0', description='LiDAR Y offset (m)')
    declare_lidar_z = DeclareLaunchArgument('lidar_z', default_value='0.15', description='LiDAR Z offset (m)')
    declare_lidar_yaw = DeclareLaunchArgument('lidar_yaw', default_value='0.0', description='LiDAR Yaw offset (rad)')

    # 1. Driver OLE LiDAR: Remap /scan -> /scan_raw
    ole_lidar_group = GroupAction([
        SetRemap(src='/scan', dst='/scan_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ole_launch_file)
        )
    ])

    # 2. Node lọc tia laser_filters: /scan_raw -> /scan
    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ]
    )

    # 3. TF tĩnh: base_link -> laser_frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', lidar_x,
            '--y', lidar_y,
            '--z', lidar_z,
            '--yaw', lidar_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame'
        ]
    )

    return LaunchDescription([
        declare_lidar_x,
        declare_lidar_y,
        declare_lidar_z,
        declare_lidar_yaw,
        ole_lidar_group,
        filter_node,
        static_tf_node
    ])

