from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quintic_trajectory_planner'

# Find config files recursively
config_files = []
for root, dirs, files in os.walk('config'):
    for f in files:
        config_files.append(os.path.join(root, f))

# Find launch files
launch_files = glob('launch/*')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        ('share/' + package_name + '/config', config_files),
        # Include launch files
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='i@todo.todo',
    description='Quintic Polynomial Trajectory Planner for ABB GoFa CRB15000',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_node = quintic_trajectory_planner.trajectory_node:main',
        ],
    },
)