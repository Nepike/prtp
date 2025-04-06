from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Получаем путь к конфигурационным файлам
    pkg_dir = get_package_share_directory('slam_esp32')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_config.yaml')

    return LaunchDescription([
        # Мост для лидара
        Node(
            package='slam_esp32',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen'
        ),

        # Мост для одометрии
        Node(
            package='slam_esp32',
            executable='odom_bridge',
            name='odom_bridge',
            output='screen'
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config],
            output='screen'
        )
    ])
