import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomous_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='daviddorina181@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'sensor_node = autonomous_robot.sensor_node:main',
        'fake_scan_publisher = autonomous_robot.fake_scan_publisher:main',
        'controller_node = autonomous_robot.controller_node:main',
        'cmd_vel_monitor = autonomous_robot.cmd_vel_monitor:main',

    ],
},



)
