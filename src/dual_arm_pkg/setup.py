from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_arm_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest', 'rclpy'],
    test_suite='test',
    maintainer='A',
    maintainer_email='a@team.example.com',
    description='Mercury X1 dual-arm driver.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dual_arm_observation_node = dual_arm_pkg.dual_arm_observation_node:main',
            'dual_gripper_server_node = dual_arm_pkg.dual_gripper_server_node:main',
            'dual_arm_pick_place_node = dual_arm_pkg.dual_arm_pick_place_node:main',
        ],
    },
)
