import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Đọc phần cứng tay cầm USB / Bluetooth
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 2. Node teleop nội bộ của project_1 (Không lo bị mất khi restart Docker)
        # Gộp cả Tiến/Lùi và Bẻ lái vào Cần Trái, Nút kích hoạt L1 (4)
        Node(
            package='project_1',
            executable='teleop_joy.py',
            name='teleop_joy_node',
            output='screen',
            parameters=[{
                'axis_linear': 1,            # Cần gạt TRÁI (Lên/Xuống): Tiến / Lùi
                'axis_angular': 0,           # Cần gạt TRÁI (Trái/Phải): Quay xe (Gộp chung cần trái)
                'scale_linear_normal': 0.8,  # Tốc độ tiến thẳng tối đa: 0.8 m/s
                'scale_angular_normal': 0.6, # Tốc độ quay vòng tối đa: 0.6 rad/s
                'scale_linear_turbo': 1.2,   # Tốc độ khi giữ Turbo (R1): 1.2 m/s
                'scale_angular_turbo': 1.0,  # Tốc độ quay Turbo: 1.0 rad/s
                'enable_deadman': True,      # Bắt buộc bấm cò mới chạy
                'btn_deadman': 4,            # Nút kích hoạt: L1 / LB
                'btn_turbo': 5,              # Nút Turbo: R1 / RB
                'btn_estop': 1,              # Nút dừng khẩn: B / Tròn
                'btn_reset_odom': 3,         # Nút reset Odometry: Y / Tam giác
            }]
        ),

        # 3. Node Gateway UDP điều khiển động cơ qua STM32 + W5500 & tính Odometry
        Node(
            package='project_1',
            executable='zlac_udp_odom_node.py',
            name='zlac_udp_odom_node',
            output='screen',
            parameters=[{
                'stm32_ip': '192.168.1.100',
                'stm32_port': 8888,
                'local_port': 8888,
                'wheel_radius': 0.0535,
                'wheel_base': 0.45,
                'publish_tf': True,
            }]
        )
    ])
