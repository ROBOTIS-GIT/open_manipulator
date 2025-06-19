import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Whether to use simulation time',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'launch_servo', default_value='false', description='Whether to launch Servo'
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='MoveGroup publishes robot description semantic',
        ),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    launch_servo = LaunchConfiguration('launch_servo')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='omx', package_name='open_manipulator_moveit_config')
        .robot_description_semantic(str(Path('config') / 'omx' / 'omx.srdf'))
        .joint_limits(str(Path('config') / 'omx' / 'joint_limits.yaml'))
        .trajectory_execution(str(Path('config') / 'omx' / 'moveit_controllers.yaml'))
        .robot_description_kinematics(str(Path('config') / 'omx' / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    servo_yaml = load_yaml('open_manipulator_moveit_config', 'config/servo.yaml')
    servo_params = {'moveit_servo': servo_yaml}
    servo_node = Node(
        package='moveit_servo',
        condition=IfCondition(launch_servo),
        executable='servo_node',
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output='screen',
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('open_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group_node,
            rviz_node,
            servo_node,
        ]
    )
