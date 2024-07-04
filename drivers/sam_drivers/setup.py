from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'sam_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share',package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Axel',
    maintainer_email='axbr@kth.se',
    description='Contains the launch files of the drivers for SAM ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sam_joint_state_converter = sam_drivers.sam_joint_state_converter:main',
            'sam_startup_check = sam_drivers.sam_startup_check:main',

        ],
    },
)
