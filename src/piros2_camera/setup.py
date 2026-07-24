import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'piros2_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # launch/ and config/ must be installed into share/<pkg>/ — ros2 launch
    # and get_package_share_directory() look there, not in the source tree.
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
    description='Milestone 3: launch file and parameters for the C922 camera.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
