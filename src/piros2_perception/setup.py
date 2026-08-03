from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'piros2_perception'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # models/ is deliberately NOT a data_file: the weights are fetched
        # after build time (`just fetch-model`) and the node resolves them
        # from the source tree via __file__ — see depth_estimator.py.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benedict Thekkel',
    maintainer_email='bthekkel1@gmail.com',
    description='Perception P1: neural monocular depth node.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # The entry point exists for discoverability, but `ros2 run` gives
            # it the system interpreter, which has no onnxruntime. The real
            # invocation runs the module under the venv python — see README.md
            # and the `just depth` recipe.
            'depth_estimator = piros2_perception.depth_estimator:main',
            'cloud_projector = piros2_perception.cloud_projector:main',
        ],
    },
)
