from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_teleop'

setup(
    name=package_name,
    version='3.2.2',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'keyboard_control_x = open_manipulator_teleop.keyboard_control_x:main',
            'keyboard_control_y = open_manipulator_teleop.keyboard_control_y:main',
            'keyboard_control_y_follower = '
            'open_manipulator_teleop.keyboard_control_y_follower:main',
        ],
    },
)
