"""Start the L100 leader and bridge only; follower impedance is launched separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('open_manipulator_joint_impedance_controller')
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('open_manipulator_bringup'), 'launch', 'omy_l100_leader_ai.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_leader')),
        launch_arguments={
            'port_name': LaunchConfiguration('port_name'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_self_collision_avoidance': LaunchConfiguration('use_self_collision_avoidance'),
        }.items())
    bridge = Node(
        package='open_manipulator_joint_impedance_controller', executable='l100_impedance_leader.py',
        name='l100_impedance_leader', output='screen',
        parameters=[LaunchConfiguration('config')])
    gripper_spawner = Node(package='controller_manager', executable='spawner', output='screen',
        condition=IfCondition(LaunchConfiguration('gripper_enabled')),
        arguments=['leader_gripper_controller',
                   '-c', '/controller_manager', '--controller-manager-timeout', '40',
                   '-p', PathJoinSubstitution([package, 'config', 'l100_gripper.yaml'])])
    gripper_bridge = Node(package='open_manipulator_joint_impedance_controller',
        executable='l100_gripper_bridge.py', output='screen',
        condition=IfCondition(LaunchConfiguration('gripper_enabled')),
        parameters=[{'use_mock_hardware': LaunchConfiguration('use_mock_hardware')}])
    return LaunchDescription([
        DeclareLaunchArgument('gripper_enabled', default_value='true'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false'),
        DeclareLaunchArgument('use_self_collision_avoidance', default_value='false'),
        DeclareLaunchArgument('start_leader', default_value='true',
                              description='false connects to an already running L100 leader'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            package, 'config', 'l100_impedance_leader.yaml'])),
        leader, bridge, gripper_spawner, gripper_bridge,
    ])
