from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'piros2_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benedict Thekkel',
    maintainer_email='bthekkel1@gmail.com',
    description='World dashboard: every feed in one window.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keypoint_detector = piros2_world.keypoint_detector:main',
            'dashboard = piros2_world.dashboard:main',
            'cloud_mapper = piros2_world.cloud_mapper:main',
        ],
    },
)
