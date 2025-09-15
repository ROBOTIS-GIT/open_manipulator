from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_teleop'

setup(
    name=package_name,
    version='4.0.9',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    author='Sungho Woo, Wonho Yun',
    author_email='wsh@robotis.com, ywh@robotis.com',
    description='OpenManipulator teleoperation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_manipulator_x_teleop = open_manipulator_teleop.open_manipulator_x_teleop:main',
            'omy_3m_teleop = open_manipulator_teleop.omy_3m_teleop:main',
            'omy_f3m_teleop = open_manipulator_teleop.omy_f3m_teleop:main',
        ],
    },
)
