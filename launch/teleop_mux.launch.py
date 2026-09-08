import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('can_test_motor')

    topics_file = os.path.join(pkg_share, 'config', 'twist_mux_topics.yaml')
    locks_file = os.path.join(pkg_share, 'config', 'twist_mux_locks.yaml')

    # 1. Node twist_mux điều phối vận tốc và an toàn
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[topics_file, locks_file],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel')],
        output='screen',
    )

    # 2. Node joy_node đọc tay cầm từ cổng /dev/input/js0
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    # 3. Node C++ xử lý logic Gamepad (Deadman L1, E-Stop B, Reset Y, Turbo R1)
    teleop_joy_node = Node(
        package='can_test_motor',
        executable='teleop_joy',
        name='teleop_joy',
        parameters=[{
            'scale_linear_normal': 0.3,
            'scale_angular_normal': 0.5,
            'scale_linear_turbo': 0.3,
            'scale_angular_turbo': 0.5,
            'enable_deadman': True,
            'btn_deadman': 10, # Nút Deadman trên tay cầm (hỗ trợ cả 10, 4, 9)
            'btn_turbo': 11,   # Nút R1 (hỗ trợ cả 11, 5)
            'btn_estop': 1,   # Nút B (tròn)
            'btn_reset_odom': 3, # Nút Y (tam giác)
        }],
        output='screen',
    )

    return LaunchDescription([
        twist_mux_node,
        joy_node,
        teleop_joy_node,
    ])