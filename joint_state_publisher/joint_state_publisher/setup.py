from setuptools import find_packages
from setuptools import setup

package_name = 'joint_state_publisher'

setup(
    name=package_name,
    version='2.12.13',
    packages=[],
    py_modules=[
        'joint_state_publisher',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='David V. Lu!!',
    author_email='davidvlu@gmail.com',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A python node to publish `sensor_msgs/JointState` messages for a '
        'robot described with URDF.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = joint_state_publisher:main',
        ],
    },
)
