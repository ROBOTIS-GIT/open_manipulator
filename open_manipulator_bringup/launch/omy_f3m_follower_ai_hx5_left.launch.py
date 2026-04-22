#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false', description='Whether to execute rviz2'),
        DeclareLaunchArgument('prefix', default_value='""', description='Prefix of the joint and link names'),
        DeclareLaunchArgument('use_sim', default_value='false', description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', description='Use mock hardware mirroring command.'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false', description='Enable mock sensor commands.'),
        DeclareLaunchArgument('init_position', default_value='false', description='Whether to launch the init_position node'),
        DeclareLaunchArgument('ros2_control_type', default_value='omy_f3m_position', description='Type of ros2_control'),
        DeclareLaunchArgument('init_position_file', default_value='initial_positions.yaml', description='Path to the initial position file'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'omy_f3m',
            'omy_f3m_hx5_left.urdf.xacro',
        ]),
        ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=', use_sim, ' ',
        'use_mock_hardware:=', use_mock_hardware, ' ',
        'mock_sensor_commands:=', mock_sensor_commands, ' ',
        'ros2_control_type:=', ros2_control_type,
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_f3m_follower_ai_hx5_left',
        'hardware_controller_manager.yaml',
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config',
        'omy_f3m_follower_ai',
        init_position_file,
    ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
        remappings=[('/arm_controller/joint_trajectory', '/leader/joint_trajectory')],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-ros-args',
            '-r /hand_l_controller/joint_trajectory:='
            '/leader/hand_l_joint_trajectory',
            'arm_controller',
            'joint_state_broadcaster',
            'hand_l_controller',
            'effort_l_controller',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='both',
    )

    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='both',
        condition=IfCondition(init_position),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='both',
        condition=IfCondition(start_rviz),
    )

    enable_synctable_process = ExecuteProcess(
        name='enable_synctable',
        cmd=[
            'bash', '-c',
            'sleep 3 && '
            'echo "[enable_synctable] Request SyncTable Enable" && '
            'result=$(ros2 service call /dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable\', item_data: 1}" 2>/dev/null) && '
            'echo "[enable_synctable] Response SyncTable Enable: $(echo "$result" | grep -o \'result=[^ ]*\')" && '
            'sleep 1 && '
            'echo "[enable_synctable] Request SyncTable Enable HX5" && '
            'result=$(ros2 service call /dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable HX5\', item_data: 1}" 2>/dev/null) && '
            'echo "[enable_synctable] Response SyncTable Enable HX5: $(echo "$result" | grep -o \'result=[^ ]*\')"',
        ],
        output='both',
    )

    left_current_command_process = ExecuteProcess(
        name='hand_l_current_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '10',
            '/effort_l_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0]',
        ],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[rviz_node])
    )
    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=robot_controller_spawner, on_exit=[joint_trajectory_executor])
    )
    delay_enable_synctable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[enable_synctable_process],
        )
    )
    delay_hand_current_command_after_enable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=enable_synctable_process,
            on_exit=[left_current_command_process],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_trajectory_executor_after_controllers,
            delay_enable_synctable,
            delay_hand_current_command_after_enable,
        ]
    )
