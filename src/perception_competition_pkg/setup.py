from glob import glob
from setuptools import find_packages, setup

package_name = 'perception_competition_pkg'

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
    install_requires=[
        'setuptools',
        'numpy<2',
        'opencv-python>=4.6,<4.10',
        'ultralytics',
    ],
    test_suite='test',
    zip_safe=True,
    maintainer='B',
    maintainer_email='b@team.example.com',
    description='YOLOE perception + drone vision alignment.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drone_target_detector = perception_competition_pkg.drone_target_detector_node:main',
            'yoloe_detector_node = perception_competition_pkg.yoloe_detector_node:main',
            'depth_pose_estimator_node = perception_competition_pkg.depth_pose_estimator_node:main',
            'detect_object_server_node = perception_competition_pkg.detect_object_server_node:main',
            'plan_to_pose_server_node = perception_competition_pkg.plan_to_pose_server_node:main',
        ],
    },
)
