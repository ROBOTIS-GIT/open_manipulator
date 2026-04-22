#!/usr/bin/env python3
#
# Test launch: OMY L100 leader + HX5-left end hardware only (no OMY F3M arm).
#
# Hardware topology
#   /dev/ttyUSB0  →  OMY END  →  HX5-D20 left hand  (follower end hardware)
#   /dev/ttyUSB1  →  OMY L100                        (leader)
#
# ROS 2 control flow
#   leader/joint_trajectory_command_broadcaster
#       → /leader/joint_trajectory
#       → omy_f3m_hx5_left_leader_trajectory_bridge  (maps rh_r1_joint to finger presets)
#       → /leader/hand_l_joint_trajectory
#       → hand_l_controller  (remapped from /hand_l_controller/joint_trajectory)
#
# Usage
#   ros2 launch open_manipulator_bringup omy_end_hx5_left_test.launch.py
#   ros2 launch open_manipulator_bringup omy_end_hx5_left_test.launch.py \
#       hand_port_name:=/dev/ttyAMA4 leader_port_name:=/dev/ttyUSB0 \
#       use_mock_hardware:=true        # bench-test without real hardware

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix applied to all joint and link names.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description=(
                'Use mock hardware for the HX5-left hand. '
                'Set true to run without physical hardware connected.'
            ),
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands (only meaningful with use_mock_hardware).',
        ),
        DeclareLaunchArgument(
            'hand_port_name',
            default_value='/dev/ttyUSB1',
            description='Serial port for the HX5-D20 left hand hardware.',
        ),
        DeclareLaunchArgument(
            'leader_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for the OMY L100 leader hardware.',
        ),
        DeclareLaunchArgument(
            'use_self_collision_avoidance',
            default_value='true',
            description='Whether to launch the self-collision detection node on the leader.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Move hand to initial position (all zeros) after controllers are active.',
        ),
    ]

    prefix = LaunchConfiguration('prefix')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    hand_port_name = LaunchConfiguration('hand_port_name')
    leader_port_name = LaunchConfiguration('leader_port_name')
    use_self_collision_avoidance = LaunchConfiguration('use_self_collision_avoidance')
    init_position = LaunchConfiguration('init_position')

    # ── Hand (end) URDF ──────────────────────────────────────────────────────
    # Standalone xacro: only the HX5-left hand links + end-unit ros2_control.
    # No OMY F3M arm hardware system is included, so the arm bus is never opened.
    hand_urdf = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('open_manipulator_description'),
                'urdf', 'omy_f3m',
                'omy_end_hx5_left_standalone.urdf.xacro',
            ]),
            ' prefix:=', prefix,
            ' use_mock_hardware:=', use_mock_hardware,
            ' mock_sensor_commands:=', mock_sensor_commands,
            ' port_name:=', hand_port_name,
        ]),
        value_type=str,
    )

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_end_hx5_left_test',
        'hardware_controller_manager.yaml',
    ])

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # ros2_control_node (Jazzy) initializes via the /robot_description topic, not
    # the parameter directly. robot_state_publisher is the single publisher of
    # /robot_description. ros2_control_node subscribes to that topic once and
    # calls init_resource_manager exactly one time.
    # Previously robot_description was also passed as a parameter to ros2_control_node,
    # which caused ros2_control_node itself to re-publish /robot_description and then
    # receive it back, triggering a second on_init mid-initialization → segfault.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': hand_urdf}],
        output='both',
    )

    hand_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_manager_config],
        output='both',
    )

    # Spawn all hand controllers in a single call so that joint_state_broadcaster
    # is guaranteed active before joint_trajectory_executor starts (OnProcessExit).
    #
    # Two remappings are packed into ONE Python string (concatenated literals) so
    # that the spawner receives them as a single --controller-ros-args value.
    # The spawner then splits on whitespace internally, producing:
    #   ['-r', '/hand_l_controller/joint_trajectory:=/leader/hand_l_joint_trajectory',
    #    '-r', '/joint_states:=/hand/joint_states']
    #
    # Passing two separate list entries (two '-r …' strings) causes argparse in
    # the spawner to consume controller names as ros-arg values → immediate fail.
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-ros-args',
            '-r /hand_l_controller/joint_trajectory:=/leader/hand_l_joint_trajectory'
            ' -r /joint_states:=/hand/joint_states',
            'hand_l_controller',
            'effort_l_controller',
            'joint_state_broadcaster',
        ],
        output='both',
    )

    # Constant effort (current) command keeps fingers compliant while position
    # commands drive the trajectory.  Values match omy_f3m_follower_ai_hx5_left.
    # Published continuously (no -t limit) so compliance persists beyond the
    # 2-second initial-position trajectory.
    hand_effort_command = ExecuteProcess(
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

    # Bridge: subscribes to /leader/joint_trajectory, extracts rh_r1_joint,
    # maps gripper value to HX5-left finger presets, publishes to
    # /leader/hand_l_joint_trajectory (→ hand_l_controller after remap above).
    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_f3m_hx5_left_leader_trajectory_bridge',
        output='both',
    )

    # Move hand to initial position (all joints = 0.0) right after controllers come up.
    init_positions_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_end_hx5_left_test',
        'initial_positions.yaml',
    ])

    hand_init_node = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[init_positions_config],
        output='both',
        condition=IfCondition(init_position),
    )

    # Start effort command, init position, and bridge only after ALL hand controllers
    # (including joint_state_broadcaster) are active. This guarantees /hand/joint_states
    # is being published before joint_trajectory_executor tries to read it.
    delay_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[hand_effort_command, hand_init_node, leader_traj_bridge],
        )
    )

    # ── Leader ────────────────────────────────────────────────────────────────
    # OMY L100 runs under the /leader namespace with its own controller_manager.
    # It publishes joint trajectories that the bridge consumes.
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('open_manipulator_bringup'),
            'launch',
            'omy_l100_leader_ai.launch.py',
        ])),
        launch_arguments={
            'port_name': leader_port_name,
            'use_self_collision_avoidance': use_self_collision_avoidance,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,  # publishes /robot_description first
            hand_control_node,           # subscribes to /robot_description to initialize
            hand_controller_spawner,
            leader_launch,
            delay_after_spawner,
        ]
    )

