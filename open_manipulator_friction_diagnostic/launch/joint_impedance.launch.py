"""Temporary external effort node; start compensation bringup separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gains', default_value=PathJoinSubstitution([
            FindPackageShare('open_manipulator_friction_diagnostic'), 'config', 'joint_impedance.yaml'])),
        Node(package='open_manipulator_friction_diagnostic', executable='joint_impedance',
             parameters=[LaunchConfiguration('gains')], output='screen',
             sigterm_timeout='5', sigkill_timeout='2')])
