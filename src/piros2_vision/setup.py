from setuptools import find_packages, setup

package_name = 'piros2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benedict Thekkel',
    maintainer_email='bthekkel1@gmail.com',
    description='Milestone 4: edge-detection node.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'edge_detector = piros2_vision.edge_detector:main',
        ],
    },
)
