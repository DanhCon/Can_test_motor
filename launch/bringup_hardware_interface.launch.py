import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('can_test_motor')

    urdf_path = os.path.join(pkg_share, 'urdf', 'zlac_robot.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    # 1. Node robot_state_publisher xuất bản cây toạ độ URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # 2. Node ros2_control_node (Controller Manager) nạp hardware interface
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
    )

    # 3. Kích hoạt bộ xuất bản trạng thái 2 khớp bánh xe (50Hz)
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

    # 4. Kích hoạt bộ điều khiển vi sai diff_drive_controller
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

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])