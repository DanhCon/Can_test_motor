import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('can_test_motor')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    amcl_config = LaunchConfiguration('amcl_config')

    # 1. Map Server: Doc va phat ban do tinh OccupancyGrid len topic /map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    # 2. AMCL: Dinh vi hat Monte Carlo so khop tia quet /scan voi ban do /map (Phat TF map -> odom)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time}
        ],
    )

    # 3. Lifecycle Manager: Tu dong kich hoat trang thai cho map_server va amcl
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'service_timeout': 10.0,
            'bond_timeout': 10.0,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Su dung thoi gian mo phong',
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(pkg_share, 'maps', 'map.yaml'),
            description='Duong dan toi file cau hinh ban do map.yaml',
        ),
        DeclareLaunchArgument(
            'amcl_config',
            default_value=os.path.join(pkg_share, 'config', 'amcl_config.yaml'),
            description='Duong dan toi file cau hinh AMCL',
        ),
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
