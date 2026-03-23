from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'block_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Block grasp task node for ROBOTIS OMY F3M',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'block_detector_debug = block_grasp.block_detector_debug_node:main',
        ],
    },
)
