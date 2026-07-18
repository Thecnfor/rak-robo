from setuptools import setup

package_name = 'isaac_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='socl',
    maintainer_email='socl@example.invalid',
    description='Isaac Sim 5.1 × ROS2 控制 starter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = isaac_ros2_control.cmd_vel_relay:main',
            'tf_echo_bridge = isaac_ros2_control.tf_echo_bridge:main',
        ],
    },
)