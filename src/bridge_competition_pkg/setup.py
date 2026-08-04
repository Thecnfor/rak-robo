from glob import glob
from setuptools import find_packages, setup

package_name = 'bridge_competition_pkg'

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
    test_suite='test',
    zip_safe=True,
    maintainer='C',
    maintainer_email='c@team.example.com',
    description='Isaac Sim ROS2 bridge: OmniGraph + host bringup.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_interface_audit = bridge_competition_pkg.interface_audit:main',
            'direct_rotor_smoke_test = bridge_competition_pkg.rotor_smoke_test:main',
            'direct_rotor_motion_probe = bridge_competition_pkg.rotor_motion_probe:main',
            'foxglove_daemon = bridge_competition_pkg.foxglove_daemon:main',
            'chain_status = bridge_competition_pkg.chain_status:main',
            'mission_trigger = bridge_competition_pkg.mission_trigger:main',
            'ground_state_sim = bridge_competition_pkg.ground_state_sim:main',
            'cargo_status_sim = bridge_competition_pkg.cargo_status_sim:main',
            'stage2_acceptance_audit = bridge_competition_pkg.stage2_acceptance:main',
            'dynamic_obstacle_probe = bridge_competition_pkg.dynamic_obstacle_probe:main',
            'dynamic_preemption_probe = bridge_competition_pkg.dynamic_preemption_probe:main',
        ],
    },
)
