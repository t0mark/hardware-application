from setuptools import setup
import os
from glob import glob

package_name = 'p_zone'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'behavior'), glob('config/behavior/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mqtt_bridge = p_zone.mqtt_bridge_node:main',
            'behavior_executor = p_zone.behavior_executor_node:main',
        ],
    },
)
