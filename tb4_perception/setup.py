from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'tb4_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qd',
    maintainer_email='quang.m.doan@gmail.com',
    description='YOLO-based object detection for TurtleBot4 autonomous navigation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = tb4_perception.yolo_detector_node:main'
        ],
    },
)
