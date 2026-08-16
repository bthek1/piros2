from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'piros2_world_mesh'

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
    description='Mesh-first fork of piros2_world: the world stack posed for the surface.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keypoint_detector = piros2_world_mesh.keypoint_detector:main',
            'dashboard = piros2_world_mesh.dashboard:main',
            'camera_relay = piros2_world_mesh.camera_relay:main',
            # Present for symmetry; the launch runs this module via the
            # perception venv python -m (open3d is PyPI-only), so the
            # colcon-shebang entry point would miss the import.
            'tsdf_mesher = piros2_world_mesh.tsdf_mesher:main',
        ],
    },
)
