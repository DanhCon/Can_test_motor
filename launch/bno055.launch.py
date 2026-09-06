import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('can_test_motor')
    default_param_file = os.path.join(pkg_dir, 'config', 'bno055_params_i2c.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Full path to the BNO055 parameters YAML file'
    )

    # 1. Driver Node BNO055
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055_node',
        output='screen',
        parameters=[LaunchConfiguration('param_file')]
    )

    # 2. Static TF Publisher: base_link -> imu_link
    # Lưu ý: Cần đo đạc lại khoảng cách vật lý thực tế trên khung xe mới
    # Thứ tự tham số: x y z yaw pitch roll frame_id child_frame_id
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        output='screen',
        arguments=['0.175', '-0.048', '0.041', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        param_file_arg,
        bno055_node,
        static_tf_node
    ])
