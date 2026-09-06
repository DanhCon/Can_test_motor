import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')

    default_ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    default_bno055_config = os.path.join(pkg_dir, 'config', 'bno055_params_i2c.yaml')

    # -------------------------------------------------------------------------
    # KHAI BAO CAC LAUNCH ARGUMENTS
    # -------------------------------------------------------------------------
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=default_ekf_config,
        description='Duong dan day du toi file cau hinh YAML cua EKF (robot_localization)'
    )

    bno055_config_arg = DeclareLaunchArgument(
        'bno055_config',
        default_value=default_bno055_config,
        description='Duong dan day du toi file cau hinh YAML cua BNO055'
    )

    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Bat/tat cum dieu khien tay cam gamepad (joy_node + teleop_joy)'
    )

    use_bno055_arg = DeclareLaunchArgument(
        'use_bno055',
        default_value='true',
        description='Bat/tat driver doc phan cung cam bien IMU BNO055 qua I2C'
    )

    # -------------------------------------------------------------------------
    # 1. DRIVER CAM BIEN IMU 9-DOF BNO055 (50Hz, I2C1, Che do NDOF da nap Offsets)
    # -------------------------------------------------------------------------
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',
        output='screen',
        parameters=[LaunchConfiguration('bno055_config')],
        condition=IfCondition(LaunchConfiguration('use_bno055'))
    )

    # Static TF Publisher: base_link -> imu_link (Toa do lap BNO055 tren robot)
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        output='screen',
        arguments=['--x', '0.175', '--y', '-0.048', '--z', '0.041',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        condition=IfCondition(LaunchConfiguration('use_bno055'))
    )

    # -------------------------------------------------------------------------
    # 2. GATEWAY DIEU KHIEN BANH XE & ODOMETRY THO (UDP 50Hz toi STM32+W5500)
    # -------------------------------------------------------------------------
    # LUU Y QUAN TRONG: publish_tf = False de tranh xung dot phat trung 2 nguon
    # TF odom -> base_link! EKF se la ben DUY NHAT phat TF nay.
    zlac_udp_node = Node(
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
            'publish_tf': False,         # <--- BAT BUOC FALSE KHI CHAY EKF
            'enable_smoother': False,    # STM32 hardware profile quan ly tang/giam toc
            'max_linear_velocity': 0.3,  # Gioi han an toan 0.3 m/s
            'max_angular_velocity': 0.8,
        }]
    )

    # -------------------------------------------------------------------------
    # 3. BO LOC DUNG HOP EXTENDED KALMAN FILTER (robot_localization)
    # -------------------------------------------------------------------------
    # Ket hop:
    #   - Van toc tinh tien vx tu Odometry banh xe (/odom)
    #   - Goc xoay Yaw tuyet doi + Van toc goc vyaw tu IMU (/bno055/imu)
    # Xuat ra:
    #   - Topic /odometry/filtered (chuan xac cao, triet tieu hien tuong truot lop)
    #   - TF chuan: odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
    )

    # -------------------------------------------------------------------------
    # 4. DIEU KHIEN TAY CAM GAMEPAD (Tuy chon, mac dinh bat de lai test thuc dia)
    # -------------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    teleop_joy_node = Node(
        package='can_test_motor',
        executable='teleop_joy',
        name='teleop_joy_node',
        output='screen',
        parameters=[{
            'axis_linear': 1,            # Can TRAI: Tien / Lui
            'axis_angular': 2,           # Can PHAI: Quay Trai / Phai
            'scale_linear_normal': 0.3,  # Toc do tien toi da: 0.3 m/s
            'scale_angular_normal': 0.5, # Toc do goc: 0.5 rad/s
            'scale_linear_turbo': 0.3,
            'scale_angular_turbo': 0.5,
            'enable_deadman': True,      # Giu nut L1 moi cho phep chay
            'btn_deadman': 4,            # L1 (tu dong nhan dien ca index 4 va 9)
            'btn_turbo': 5,              # R1
            'btn_estop': 1,              # Nut dung khan: B / Tron
            'btn_reset_odom': 3,         # Nut reset Odometry: Y / Tam giac
        }],
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    return LaunchDescription([
        ekf_config_arg,
        bno055_config_arg,
        use_joy_arg,
        use_bno055_arg,
        bno055_node,
        static_tf_imu,
        zlac_udp_node,
        ekf_node,
        joy_node,
        teleop_joy_node
    ])
