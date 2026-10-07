from setuptools import setup
import os
from glob import glob

package_name = 'sqp_planner'

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
    description='SQP multi-obstacle avoidance planner ported from unicorn-racing-stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sqp_avoidance_node = sqp_planner.sqp_avoidance_node:main',
            'update_waypoints = sqp_planner.update_waypoints:main',
        ],
    },
)
