from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_project_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zunibui',
    maintainer_email='buidungitmo2227@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_data_proc = ros_project_scene.sensor_data_proc:main',
            # 'scene_publisher = ros_project_scene.scene_publisher:main',
            'move_straight = ros_project_scene.move_straight:main',
            'control_robot = ros_project_scene.control_robot:main',
        ],
    },
)
