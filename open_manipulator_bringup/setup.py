from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_bringup'

setup(
    name=package_name,
    version='4.0.9',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config/open_manipulator_x'),
            glob('config/open_manipulator_x/*')),
        (os.path.join('share', package_name, 'config/omy_3m'), glob('config/omy_3m/*')),
        (os.path.join('share', package_name, 'config/omy_f3m'), glob('config/omy_f3m/*')),
        (os.path.join('share', package_name, 'config/omy_f3m_follower_ai'),
            glob('config/omy_f3m_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omy_f3m_leader_ai'),
            glob('config/omy_f3m_leader_ai/*')),
        (os.path.join('share', package_name, 'config/omy_l100_follower_ai'),
            glob('config/omy_l100_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omy_l100_leader_ai'),
            glob('config/omy_l100_leader_ai/*')),
        (os.path.join('share', package_name, 'config/open_manipulator_x'),
            glob('config/open_manipulator_x/*')),
        (os.path.join('share', package_name, 'config/omx_f'),
            glob('config/omx_f/*')),
        (os.path.join('share', package_name, 'config/omx_f_follower_ai'),
            glob('config/omx_f_follower_ai/*')),
        (os.path.join('share', package_name, 'config/omx_l_leader_ai'),
            glob('config/omx_l_leader_ai/*')),
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
            'om_create_udev_rules = open_manipulator_bringup.om_create_udev_rules:main',
        ],
    },
)
