#!/usr/bin/env python3
"""
==============================================================================
Launch file tổng hợp toàn diện (All-in-One Bringup) cho robot AMR:
  1. robot.launch.py:
     - ros2_control (ZlacHardwareInterface C++ kết nối STM32 W5500)
     - Joint State Broadcaster + Diff Drive Controller (50Hz)
     - IMU BNO055 (50Hz I2C) + Static TF
     - EKF robot_localization (20Hz dung hợp odom + imu)
     - OLE LiDAR (15Hz Ethernet UDP) + laser_filters (gọt góc sau xe +-119 độ)
     - Gamepad teleop (teleop_joy C++) + twist_mux (ưu tiên Joy > Key > Nav)
  2. amcl.launch.py (tùy chọn use_amcl, mặc định true):
     - map_server (nạp bản đồ tĩnh)
     - AMCL (định vị hạt Monte Carlo, phát TF map -> odom)
     - Lifecycle Manager Localization
  3. nav2.launch.py (tùy chọn use_nav2, mặc định true):
     - Controller Server (MPPI / DWB bám quỹ đạo)
     - Planner Server (NavFn / Smac lập đường đi toàn cục)
     - BT Navigator + Recovery Behaviors
     - Lifecycle Manager Navigation

Cách dùng:
  # Chạy toàn bộ tự hành (Hardware + AMCL + Nav2):
  ros2 launch can_test_motor bringup_all.launch.py

  # Chỉ chạy phần cứng + định vị AMCL (không bật bộ dẫn đường Nav2):
  ros2 launch can_test_motor bringup_all.launch.py use_nav2:=false

  # Chỉ chạy phần cứng để test tay cầm độc lập:
  ros2 launch can_test_motor robot.launch.py
==============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('can_test_motor')

    use_amcl = LaunchConfiguration('use_amcl')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_joy = LaunchConfiguration('use_joy')
    enable_deadman = LaunchConfiguration('enable_deadman')

    # Launch Arguments
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl',
        default_value='true',
        description='Bật/tắt Map Server và định vị AMCL (true/false)'
    )

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Bật/tắt cụm điều hướng Nav2 (true/false)'
    )

    declare_use_joy = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Bật/tắt cụm tay cầm điều khiển Gamepad (true/false)'
    )

    declare_enable_deadman = DeclareLaunchArgument(
        'enable_deadman',
        default_value='false',
        description='Bắt buộc giữ phím L1 để chạy xe (true/false, mặc định false để tiện test)'
    )

    # 1. Cụm nền tảng phần cứng & cảm biến (robot.launch.py)
    robot_launch_file = os.path.join(pkg_share, 'launch', 'robot.launch.py')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_file),
        launch_arguments={
            'use_joy': use_joy,
            'enable_deadman': enable_deadman,
            'use_bno055': 'true',
            'use_lidar': 'true',
            'use_ekf': 'true',
        }.items()
    )

    # 2. Cụm bản đồ & định vị AMCL (amcl.launch.py)
    amcl_launch_file = os.path.join(pkg_share, 'launch', 'amcl.launch.py')
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_file),
        condition=IfCondition(use_amcl)
    )

    # 3. Cụm dẫn đường tự hành Nav2 (nav2.launch.py)
    nav2_launch_file = os.path.join(pkg_share, 'launch', 'nav2.launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        condition=IfCondition(use_nav2)
    )

    return LaunchDescription([
        declare_use_amcl,
        declare_use_nav2,
        declare_use_joy,
        declare_enable_deadman,
        robot_launch,
        amcl_launch,
        nav2_launch
    ])
