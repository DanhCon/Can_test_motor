import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Đường dẫn tới file config/joy.yaml trong package project_1
    try:
        pkg_dir = get_package_share_directory('project_1')
        joy_config = os.path.join(pkg_dir, 'config', 'joy.yaml')
    except Exception:
        # Đường dẫn dự phòng cục bộ nếu chưa cài đặt share
        joy_config = os.path.join(os.getcwd(), 'src', 'project_1', 'config', 'joy.yaml')

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

        # 2. Node chuẩn teleop_twist_joy nạp trực tiếp file config/joy.yaml
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config]
        ),

        # 3. Node Gateway UDP điều khiển động cơ qua STM32 + W5500
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
