from setuptools import setup
import os
from glob import glob

package_name = 'goal_path_fixer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Fixes Path and GoalPose headers to refresh timestamps',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fix_path_header = goal_path_fixer.fix_path_header:main',
            'goal_pose_refresher = goal_path_fixer.goal_pose_refresher:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
)
