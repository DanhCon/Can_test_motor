import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node đọc phần cứng tay cầm USB / Bluetooth
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

        # 2. Node ánh xạ tay cầm sang /cmd_vel (Deadman L1, Turbo R1, E-stop B)
        Node(
            package='can_test_motor',
            executable='teleop_joy',
            name='teleop_joy_node',
            output='screen',
            parameters=[{
                'axis_linear': 1,            # Cần gạt trái Y: Tiến / Lùi
                'axis_angular': 3,           # Cần gạt phải X: Rẽ Trái / Phải
                'scale_linear_normal': 0.3,  # Tốc độ an toàn: 0.3 m/s
                'scale_angular_normal': 0.5, # Tốc độ góc an toàn: 0.5 rad/s
                'scale_linear_turbo': 0.3,   # Tốc độ Turbo khóa ở 0.3 m/s
                'scale_angular_turbo': 0.5,  # Tốc độ góc Turbo: 0.5 rad/s
                'enable_deadman': True,      # Giữ nút L1 (LB) để chạy
                'btn_deadman': 4,            # L1 / LB (nhận diện cả index 4 và 9)
                'btn_turbo': 5,              # R1 / RB
                'btn_estop': 1,              # B / Tròn
                'btn_reset_odom': 3,         # Y / Tam giác
            }]
        ),

        # 3. Node Gateway UDP điều khiển động cơ và tính Odometry
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
                'enable_smoother': False,
                'max_linear_velocity': 0.3,
                'max_angular_velocity': 0.8,
            }]
        )
    ])
