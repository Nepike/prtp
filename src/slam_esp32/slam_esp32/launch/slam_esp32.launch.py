from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_nav',
            executable='lidar_bridge',
            name='lidar_bridge'
        ),
        Node(
            package='robot_nav',
            executable='odom_bridge',
            name='odom_bridge'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{'params_file': '/slam_config.yaml'}]
        ),
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            parameters=[{'params_file': '/nav2_params.yaml'}]
        )
    ])
