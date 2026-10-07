from setuptools import setup
import os
from glob import glob

package_name = 'lane_change_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skkai',
    maintainer_email='hwany01@g.skku.edu',
    description='Lane-change overtaking planner ported from unicorn-racing-stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'change_avoidance_node = lane_change_planner.change_avoidance_node:main',
        ],
    },
)
