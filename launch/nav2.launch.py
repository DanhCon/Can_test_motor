import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('can_test_motor')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_dir = os.path.join(pkg_share, 'config', 'nav2')

    controller_yaml = os.path.join(params_dir, 'controller.yaml')
    planner_yaml = os.path.join(params_dir, 'planner_server.yaml')
    recovery_yaml = os.path.join(params_dir, 'recovery.yaml')
    bt_navigator_yaml = os.path.join(params_dir, 'bt_navigator.yaml')

    common_params = {'use_sim_time': use_sim_time}

    # 1. Controller Server: Bam quy dao cuc bo (MPPI / DWB Controller)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml, common_params],
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    # 2. Planner Server: Lap duong di toan cuc (Theta* / NavFn Planner)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml, common_params],
    )

    # 3. Behavior Server: Cac hanh vi phuc hoi (Spin, Backup, Wait khi bi ket)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[recovery_yaml, common_params],
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    # 4. BT Navigator: Dieu phoi cay hanh vi (Behavior Tree Navigator)
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml, common_params],
    )

    # 5. Lifecycle Manager: Tu dong kich hoat toan bo cac node Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'service_timeout': 10.0,
            'bond_timeout': 10.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Su dung thoi gian mo phong',
        ),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
