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
            package='can_test_motor',
            executable='teleop_joy',
            name='teleop_joy_node',
            output='screen',
            parameters=[{
                'axis_linear': 1,            # Cần TRÁI (Lên/Xuống): Tiến / Lùi
                'axis_angular': 2,           # Cần PHẢI (Trái/Phải): Bẻ lái Quay xe
                'scale_linear_normal': 0.3,  # Giới hạn tốc độ tiến thẳng: tối đa 0.3 m/s
                'scale_angular_normal': 0.5, # Tốc độ quay xe: 0.5 rad/s
                'scale_linear_turbo': 0.3,   # Turbo khóa ở 0.3 m/s
                'scale_angular_turbo': 0.5,  # Tốc độ quay Turbo: 0.5 rad/s
                'enable_deadman': True,      # Bắt buộc giữ L1 mới cho chạy
                'btn_deadman': 4,            # Nút kích hoạt: L1 (vị trí số 5 = index 4)
                'btn_turbo': 5,              # Nút Turbo: R1
                'btn_estop': 1,              # Nút dừng khẩn: B / Tròn
                'btn_reset_odom': 3,         # Nút reset Odometry: Y / Tam giác
            }]
        ),

        # 3. Node Gateway UDP điều khiển động cơ qua STM32 + W5500 & tính Odometry
        Node(
            package='can_test_motor',
            executable='zlac_udp_odom_node',
            name='zlac_udp_odom_node',
            output='screen',
            parameters=[{
                'stm32_ip': '192.168.1.100',
                'stm32_port': 8888,
                'local_port': 8888,
                'wheel_radius': 0.0535,
                'wheel_base': 0.45,
                'publish_tf': True,
                'enable_smoother': False,    # Tắt tăng giảm tốc mềm ROS 2 (để phần cứng STM32 tự lo)
                'max_linear_velocity': 0.3,  # Giới hạn an toàn 0.3 m/s
                'max_angular_velocity': 0.8,
            }]
        )
    ])
