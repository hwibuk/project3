from setuptools import setup
import os
from glob import glob

package_name = 'pick_place_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwibuk',
    maintainer_email='hwibuk@todo.todo',
    description='Pick and place with moveit_py',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_target_controller = pick_place_pkg.pose_target_controller:main',
        ],
    },
)
