from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    slam_config = os.path.join(robot_navigation_dir, 'config', 'slam_config.yaml')
    nav2_params = os.path.join(robot_navigation_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='online_async_robot_node',
            name='slam_toolbox',
            parameters=[slam_config],
            output='screen'
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']
            ),
            launch_arguments={'params_file': nav2_params}.items()
        ),

        # Мост для данных с ESP32
        Node(
            package='robot_navigation',
            executable='robot_bridge',
            name='robot_bridge',
            output='screen'
        ),

        # Отправитель команд на робота
        Node(
            package='robot_navigation',
            executable='cmd_vel_sender',
            name='cmd_vel_sender',
            output='screen'
        )
    ])
