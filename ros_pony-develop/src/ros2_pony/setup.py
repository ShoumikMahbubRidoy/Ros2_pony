from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_pony'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('ros2_pony/msg/*.msg')),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='k-okina-d1',
    maintainer_email='k-okina-d1@sivax.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processor_node = ros2_pony.processor_node:main',
            'drive_usb2can_node = ros2_pony.drive_usb2can_node:main',
            'joint_can_converter_node = ros2_pony.joint_can_converter_node:main',
            'motion_planner_node = ros2_pony.motion_planner_node:main',
        ],
    },
    # Add dependency on generated messages
    requires=['ros2_pony']
)
