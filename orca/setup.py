from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orca'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel1127',
    maintainer_email='samuel11271127@gmail.com',
    description='Orca',
    license='DO NOT USE WHEN YOURE NOT A STUDENT OF SNU NAOE',
    entry_points={
        'console_scripts': [
            'pose = orca.pose:main',
            'waypoint = orca.waypoint:main',
            'autopilot = orca.autopilot:main',
            'motion = orca.motion:main',
        ],
    },
)
