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

        # 2. Node teleop_joy: Cần TRÁI lái Tiến/Lùi, Cần PHẢI bẻ lái Trái/Phải
        Node(
            package='project_1',
            executable='teleop_joy.py',
            name='teleop_joy_node',
            output='screen',
            parameters=[{
                'axis_linear': 1,            # Cần TRÁI (Lên/Xuống): Tiến / Lùi (Dòng số 2 trong axes)
                'axis_angular': 2,           # Cần PHẢI (Trái/Phải): Bẻ lái Quay xe (Dòng số 3 trong axes)
                'scale_linear_normal': 0.8,  # Tốc độ tiến thẳng: 0.8 m/s
                'scale_angular_normal': 0.8, # Tốc độ quay xe: 0.8 rad/s
                'scale_linear_turbo': 1.2,   # Tốc độ khi giữ Turbo (R1): 1.2 m/s
                'scale_angular_turbo': 1.2,  # Tốc độ quay Turbo: 1.2 rad/s
                'enable_deadman': True,      # Bắt buộc bấm cò mới chạy
                'btn_deadman': 9,            # Nút kích hoạt: L1 (Dòng số 10 trong buttons)
                'btn_turbo': 5,              # Nút Turbo: R1
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
