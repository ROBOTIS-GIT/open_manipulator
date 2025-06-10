from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_bringup'

setup(
    name=package_name,
    version='3.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config/om_x'), glob('config/om_x/*')),
        (os.path.join('share', package_name, 'config/om_y'), glob('config/om_y/*')),
        (
            os.path.join('share', package_name, 'config/om_y_follower'),
            glob('config/om_y_follower/*'),
        ),
        (
            os.path.join('share', package_name, 'config/om_y_leader'),
            glob('config/om_y_leader/*'),
        ),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name), ['open-manipulator-cdc.rules']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='OpenMANIPULATOR bringup ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_executor = open_manipulator_bringup.joint_trajectory_executor:main',
            'pack_unpack_y = open_manipulator_bringup.pack_unpack_y:main',
            'om_create_udev_rules = open_manipulator_bringup.om_create_udev_rules:main',
        ],
    },
)
