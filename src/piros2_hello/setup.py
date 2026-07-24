from setuptools import find_packages, setup

package_name = 'piros2_hello'

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
    description='Milestone 1: a hand-written timer publisher and logging subscriber.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # Each line makes `ros2 run piros2_hello <name>` work: colcon installs a
    # console script named <name> that calls <module>:<function>.
    entry_points={
        'console_scripts': [
            'talker = piros2_hello.talker:main',
            'listener = piros2_hello.listener:main',
        ],
    },
)
