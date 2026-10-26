import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asvin',
    maintainer_email='asvin@todo.todo',
    description='Platform-independent ROS 2 Swarm API and launch configurations',
    license='MIT',
    entry_points={
        'console_scripts': [
            'single_drone = swarm.drone:main',
        ],
    },
)