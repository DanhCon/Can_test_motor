import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Tự động dọn dẹp file khóa Shared Memory cũ của FastDDS để tránh lỗi open_and_lock_file failed
    for f in glob.glob('/dev/shm/fastrtps*') + glob.glob('/dev/shm/sem.fastrtps*'):
        try:
            os.remove(f)
        except Exception:
            pass

    pkg_dir = get_package_share_directory('can_test_motor')

    # Đường dẫn các file cấu hình và mô hình URDF
    urdf_path = os.path.join(pkg_dir, 'urdf', 'zlac_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'diff_drive_controller.yaml')
    topics_file = os.path.join(pkg_dir, 'config', 'twist_mux_topics.yaml')
    locks_file = os.path.join(pkg_dir, 'config', 'twist_mux_locks.yaml')
    default_ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    default_bno055_config = os.path.join(pkg_dir, 'config', 'bno055_params_i2c.yaml')

    # -------------------------------------------------------------------------
    # KHAI BÁO CÁC LAUNCH ARGUMENTS
    # -------------------------------------------------------------------------
    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Bật/tắt cụm tay cầm Gamepad (joy_node + teleop_joy)'
    )
    enable_deadman_arg = DeclareLaunchArgument(
        'enable_deadman',
        default_value='true',
        description='Bật/tắt chế độ giữ nút an toàn Deadman (false: không cần giữ nút)'
    )
    use_bno055_arg = DeclareLaunchArgument(
        'use_bno055',
        default_value='true',
        description='Bật/tắt cảm biến IMU BNO055 qua giao tiếp I2C'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Bật/tắt bộ lọc dung hợp Extended Kalman Filter (robot_localization)'
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Bật/tắt cảm biến quét môi trường OLE LiDAR qua Ethernet UDP'
    )
    ekf_config_arg = DeclareLaunchArgument(
        'ekf_config',
        default_value=default_ekf_config,
        description='Đường dẫn file cấu hình YAML của EKF'
    )
    bno055_config_arg = DeclareLaunchArgument(
        'bno055_config',
        default_value=default_bno055_config,
        description='Đường dẫn file cấu hình YAML của BNO055'
    )

    # -------------------------------------------------------------------------
    # 1. PHẦN CỨNG ROS2_CONTROL CHO ĐỘNG CƠ ZLAC8015D (UDP QUA STM32 + W5500)
    # -------------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    # Node robot_state_publisher xuất bản URDF cây toạ độ robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Node ros2_control_node (Controller Manager) nạp hardware interface ZLAC UDP
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
    )

    # Spawner kích hoạt joint_state_broadcaster (50Hz)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
        ],
        output='screen',
    )

    # Spawner kích hoạt diff_drive_controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
        ],
        output='screen',
    )

    # Khởi động tuần tự: joint_state_broadcaster sau 3s, xong mới tới diff_drive_controller
    delay_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delay_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # -------------------------------------------------------------------------
    # 2. BỘ ĐIỀU PHỐI VẬN TỐC & AN TOÀN TWIST_MUX
    # -------------------------------------------------------------------------
    # Đầu vào ưu tiên: Joy (/input_joy/cmd_vel: 99), Key (/key_vel: 90), Nav2 (/cmd_vel: 10)
    # Khóa an toàn: /safety_stop (priority: 255)
    # Đầu ra điều khiển: /diff_drive_controller/cmd_vel_unstamped (chuẩn Twist)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[topics_file, locks_file],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # -------------------------------------------------------------------------
    # 3. TAY CẦM ĐIỀU KHIỂN GAMEPAD (JOY_NODE + TELEOP_JOY C++)
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
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    teleop_joy_node = Node(
        package='can_test_motor',
        executable='teleop_joy',
        name='teleop_joy_node',
        output='screen',
        parameters=[{
            'axis_linear': 1,            # Cần gạt trái: Tiến / Lùi (vị trí số 2)
            'axis_angular': 2,           # Cần gạt phải: Quay Trái / Phải (vị trí số 3)
            'scale_linear_normal': 0.3,  # Giới hạn an toàn 0.3 m/s
            'scale_angular_normal': 0.5, # Giới hạn góc 0.5 rad/s
            'scale_linear_turbo': 0.3,
            'scale_angular_turbo': 0.5,
            'enable_deadman': LaunchConfiguration('enable_deadman'),
            'btn_deadman': 10,           # L1 (hỗ trợ cả 10, 4, 9)
            'btn_turbo': 11,             # R1 (hỗ trợ cả 11, 5)
            'btn_estop': 1,              # Nút B (tròn): Dừng khẩn cấp
            'btn_reset_odom': 3,         # Nút Y (tam giác): Reset
        }],
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    # -------------------------------------------------------------------------
    # 4. CẢM BIẾN IMU 9-DOF BNO055 (I2C1 JETSON TX2, NDOF MODE)
    # -------------------------------------------------------------------------
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',
        output='screen',
        parameters=[LaunchConfiguration('bno055_config')],
        condition=IfCondition(LaunchConfiguration('use_bno055')),
    )

    # Static TF Publisher: base_link -> imu_link (Toạ độ lắp BNO055 trên robot)
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        output='screen',
        arguments=['--x', '0.175', '--y', '-0.048', '--z', '0.041',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        condition=IfCondition(LaunchConfiguration('use_bno055')),
    )

    # -------------------------------------------------------------------------
    # 5. BỘ LỌC DUNG HỢP EXTENDED KALMAN FILTER (robot_localization)
    # -------------------------------------------------------------------------
    # Dung hợp vx từ /diff_drive_controller/odom + Yaw/vyaw từ /bno055/imu
    # Phát ra TF: odom -> base_link và topic /odometry/filtered
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    # -------------------------------------------------------------------------
    # 6. CẢM BIẾN QUÉT MÔI TRƯỜNG OLE LIDAR (ETHERNET UDP)
    # -------------------------------------------------------------------------
    # Static TF Publisher: base_link -> laser_frame (Vị trí lắp OLE LiDAR trên robot)
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=['--x', '0.20', '--y', '0.0', '--z', '0.15',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    # Driver node OLE LiDAR (gói ros2_lidar)
    ole_lidar_launch_file = os.path.join(get_package_share_directory('ros2_lidar'), 'launch', 'ole2dv2_launch.py')
    ole_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ole_lidar_launch_file),
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    return LaunchDescription([
        use_joy_arg,
        enable_deadman_arg,
        use_bno055_arg,
        use_ekf_arg,
        use_lidar_arg,
        ekf_config_arg,
        bno055_config_arg,

        # 1. ros2_control hardware interface
        robot_state_publisher,
        control_node,
        delay_joint_state_broadcaster,
        delay_diff_drive_controller,

        # 2. twist_mux
        twist_mux_node,

        # 3. Gamepad teleop
        joy_node,
        teleop_joy_node,

        # 4. IMU
        bno055_node,
        static_tf_imu,

        # 5. EKF Fusion
        ekf_node,

        # 6. LiDAR
        static_tf_laser,
        ole_lidar_launch,
    ])
