"""Headless Gazebo and video infrastructure used by calibrate.launch.py."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from open_manipulator_friction_diagnostic.simulation import simulation_actions


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    return simulation_actions(value('output'), Path(value('urdf')).read_text(),
                              real_time_factor=float(value('real_time_factor')),
                              video_fps=float(value('video_fps')))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output'), DeclareLaunchArgument('urdf'),
        DeclareLaunchArgument('real_time_factor', default_value='10.0'),
        DeclareLaunchArgument('video_fps', default_value='5.0'),
        OpaqueFunction(function=setup)])
