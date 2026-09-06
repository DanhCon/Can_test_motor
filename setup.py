import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'can_test_motor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hocho',
    maintainer_email='hocho@todo.todo',
    description='ROS 2 Gateway and Teleop for STM32 W5500 ZLAC8015D and BNO055',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zlac_udp_odom_node = scripts.zlac_udp_odom_node:main',
            'teleop_joy = scripts.teleop_joy:main',
        ],
    },
)
