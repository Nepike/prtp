from setuptools import find_packages, setup

package_name = 'slam_esp32'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nepike',
    maintainer_email='nepike@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'robot_bridge = robot_navigation.robot_bridge:main',
            'cmd_vel_sender = robot_navigation.cmd_vel_sender:main',
        ],
    },
    },
)
