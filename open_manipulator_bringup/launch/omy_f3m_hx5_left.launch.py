#!/usr/bin/env python3
"""
omy_f3m_hx5_left.launch.py

Combined teleoperation launch:
  * F3M follower arm (joint1-6)             ← driven by /leader/joint_trajectory
  * HX5 left hand via OMY END SyncTable     ← driven by leader_trajectory_bridge
  * L100 leader (publishes /leader/...)

Topology
  /dev/ttyUSB0 (default) : OMY END (HX5 hand, hub ID 210)
  /dev/ttyUSB1 (default) : L100 leader
  F3M follower arm uses the port hard-coded inside its own xacro / udev rule
  (matches the standalone omy_f3m.launch.py behavior).

Sequencing (HX5 first, follower after — mirrors omy_end_hx5_left_l100.launch.py)
  1. control nodes for HX5 (in /hand namespace) and follower arm spin up
  2. T = 5 s : spawn HX5 controllers
  3. HX5 spawner exits  → start hand current command (Goal Current = 300)
                       → enable SyncTable / SyncTable HX5 via set_dxl_data
                         (3 s internal sleep so BulkWrite already carries
                         Goal Current = 300 before the relay starts)
  4. SyncTable enable exits → 2 s delay → send HX5 initial pose
  5. HX5 initial pose exits → spawn follower controllers (arm only,
                              gripper_controller intentionally NOT spawned —
                              the leader trigger drives HX5 instead)
  6. follower spawner exits → joint_trajectory_executor (init pose) +
                              leader_trajectory_bridge (HX5 trigger ready) +
                              rviz (optional)

SyncTable activation
  Same as omy_end_hx5_left_l100.launch.py: this launch DOES auto-call
  set_dxl_data for SyncTable Enable and SyncTable Enable HX5 (id 210). If
  you have already pre-enabled them in Dynamixel Wizard the calls are
  idempotent (just re-write 1).

Usage
  ros2 launch open_manipulator_bringup omy_f3m_hx5_left.launch.py \
      hand_port_name:=/dev/ttyUSB0 \
      leader_port_name:=/dev/ttyUSB1 \
      use_self_collision_avoidance:=false
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'hand_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for OMY END (HX5 hand, hub ID 210).',
        ),
        DeclareLaunchArgument(
            'leader_port_name',
            default_value='/dev/ttyUSB1',
            description='Serial port for OMY L100 leader.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for both HX5 and follower arm.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands for the follower arm.',
        ),
        DeclareLaunchArgument(
            'use_self_collision_avoidance',
            default_value='true',
            description='Enable self-collision detection on the leader.',
        ),
        DeclareLaunchArgument(
            'leader_use_sim',
            default_value='false',
            description='If true, pass use_sim:=true to the leader launch.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Run the follower arm in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Move the follower arm to its initial pose after spawn.',
        ),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='init_position_hx5.yaml',
            description='Initial pose YAML under config/omy_f3m/.',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='omy_f3m_position',
            description='Type of ros2_control for the follower arm.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix for the follower arm joint and link names.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to start rviz2.',
        ),
    ]
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    hand_port_name = LaunchConfiguration('hand_port_name').perform(context)
    leader_port_name = LaunchConfiguration('leader_port_name')
    use_mock_hardware_str = LaunchConfiguration('use_mock_hardware').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    use_self_collision_avoidance = LaunchConfiguration('use_self_collision_avoidance')
    leader_use_sim = LaunchConfiguration('leader_use_sim')
    use_sim = LaunchConfiguration('use_sim')
    init_position = LaunchConfiguration('init_position')
    init_position_file = LaunchConfiguration('init_position_file')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    prefix = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')

    # ─── HX5 hand subsystem (namespace /hand) ───────────────────────────
    # URDF is materialized at launch time so the controller_manager gets
    # robot_description as a concrete string parameter (avoids DDS races
    # against stale topic publishers from previous runs).
    hand_xacro = os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'urdf', 'omy_f3m', 'omy_end_hx5_left_standalone.urdf.xacro')
    hand_urdf_str = subprocess.check_output([
        'xacro', hand_xacro,
        f'port_name:={hand_port_name}',
        f'use_mock_hardware:={use_mock_hardware_str}',
    ]).decode('utf-8')

    hand_controller_manager_config = os.path.join(
        get_package_share_directory('open_manipulator_bringup'),
        'config', 'omy_end_hx5_left_l100',
        'hardware_controller_manager.yaml')

    hand_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': hand_urdf_str}],
        remappings=[
            ('robot_description', '/hand_robot_description'),
            ('joint_states', '/hand/joint_states'),
        ],
        output='both',
    )

    hand_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='hand',
        parameters=[
            {'robot_description': hand_urdf_str},
            hand_controller_manager_config,
        ],
        remappings=[('robot_description', '/hand_robot_description')],
        output='both',
        condition=UnlessCondition(leader_use_sim),
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'hand_l_controller',
            'effort_l_controller',
            '-c', '/hand/controller_manager',
        ],
        output='both',
    )

    hand_current_command_process = ExecuteProcess(
        name='hand_l_current_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '10',
            '/hand/effort_l_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0]',
        ],
    )

    # Goal Current=300 must be in BulkWrite data BEFORE SyncTable relay starts.
    # Otherwise HX5 receives Goal Current=0 → torque drops immediately.
    enable_synctable_process = ExecuteProcess(
        name='enable_synctable',
        cmd=[
            'bash', '-c',
            'echo "[enable_synctable] Waiting for Goal Current to propagate..." && '
            'sleep 3 && '
            'echo "[enable_synctable] Activating SyncTable Enable..." && '
            'ros2 service call /hand/dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable\', item_data: 1}" && '
            'sleep 1 && '
            'echo "[enable_synctable] Activating SyncTable Enable HX5..." && '
            'ros2 service call /hand/dynamixel_hardware_interface/set_dxl_data '
            'dynamixel_interfaces/srv/SetDataToDxl '
            '"{id: 210, item_name: \'SyncTable Enable HX5\', item_data: 1}" && '
            'echo "[enable_synctable] Done."',
        ],
        output='both',
    )

    hand_initial_pose_process = ExecuteProcess(
        name='hand_l_initial_pose',
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/hand/hand_l_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{'
            'joint_names: ['
            'finger_l_joint1, finger_l_joint2, finger_l_joint3, finger_l_joint4, '
            'finger_l_joint5, finger_l_joint6, finger_l_joint7, finger_l_joint8, '
            'finger_l_joint9, finger_l_joint10, finger_l_joint11, finger_l_joint12, '
            'finger_l_joint13, finger_l_joint14, finger_l_joint15, finger_l_joint16, '
            'finger_l_joint17, finger_l_joint18, finger_l_joint19, finger_l_joint20], '
            'points: [{positions: ['
            '0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0], '
            'time_from_start: {sec: 3, nanosec: 0}}]'
            '}',
        ],
        output='both',
    )

    # ─── F3M follower arm subsystem ─────────────────────────────────────
    # Mirrors omy_f3m.launch.py but: (a) remaps /arm_controller/joint_trajectory
    # to /leader/joint_trajectory so the follower tracks the leader, and
    # (b) does NOT spawn gripper_controller (the leader trigger drives HX5
    # via the bridge instead).
    arm_urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf', 'omy_f3m', 'omy_f3m.urdf.xacro',
        ]),
        ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=', use_sim, ' ',
        'use_mock_hardware:=', use_mock_hardware, ' ',
        'mock_sensor_commands:=', mock_sensor_commands, ' ',
        'ros2_control_type:=', ros2_control_type,
    ])

    arm_controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_f3m', 'hardware_controller_manager.yaml',
    ])

    arm_trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omy_f3m', init_position_file,
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz', 'open_manipulator.rviz',
    ])

    arm_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': arm_urdf_file, 'use_sim_time': use_sim}],
        output='both',
    )

    arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': arm_urdf_file}, arm_controller_manager_config],
        remappings=[('/arm_controller/joint_trajectory', '/leader/joint_trajectory')],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': arm_urdf_file}],
    )

    arm_init_position_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[arm_trajectory_params_file],
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

    # ─── L100 leader ────────────────────────────────────────────────────
    # GroupAction + SetRemap shields the leader's controller_manager from
    # any stale /robot_description topic left over from prior runs.
    leader_launch = GroupAction([
        SetRemap('robot_description', 'leader_robot_description_internal'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('open_manipulator_bringup'),
                'launch', 'omy_l100_leader_ai.launch.py',
            )),
            launch_arguments=[
                ('port_name', leader_port_name),
                ('use_self_collision_avoidance', use_self_collision_avoidance),
                ('use_sim', leader_use_sim),
                # Override the parent launch's ros2_control_type
                # (which is omy_f3m_position for the follower arm) so the
                # leader xacro picks up its own omy_l100_current variant
                # and finds xacro:omy_l100_system.
                ('ros2_control_type', 'omy_l100_current'),
            ],
        ),
    ])

    # ─── Bridge: leader rh_r1_joint → HX5 finger presets ────────────────
    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_f3m_hx5_left_leader_trajectory_bridge',
        parameters=[
            {'leader_trajectory_topic': '/leader/joint_trajectory'},
            {'hand_command_topic': '/hand/hand_l_controller/joint_trajectory'},
        ],
        output='both',
    )

    # ─── Sequencing (mirrors omy_end_hx5_left_l100.launch.py) ───────────
    # Step 1: 5 s after launch → spawn HX5 controllers.
    delayed_hand_spawner = TimerAction(
        period=5.0,
        actions=[hand_controller_spawner],
    )

    # Step 2a: HX5 spawner done → start the constant Goal Current=300
    # publisher so the BulkWrite buffer carries non-zero current before
    # SyncTable relay starts.
    start_hand_current_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[hand_current_command_process],
        )
    )

    # Step 2b: HX5 spawner done → enable SyncTable + SyncTable HX5
    # (3 s internal delay so BulkWrite already contains Goal Current=300
    # before relay starts).
    start_synctable_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[enable_synctable_process],
        )
    )

    # Step 3: SyncTable enabled → 2 s later send HX5 initial pose.
    delayed_hand_initial_pose = TimerAction(
        period=2.0,
        actions=[hand_initial_pose_process],
    )
    start_hand_initial_after_synctable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=enable_synctable_process,
            on_exit=[delayed_hand_initial_pose],
        )
    )

    # Step 4: HX5 initial pose done → spawn follower arm controllers.
    # This guarantees the hand is already torqued and at home before the
    # arm starts moving.
    start_arm_after_hand_initial = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_initial_pose_process,
            on_exit=[arm_controller_spawner],
        )
    )

    # Step 5: arm spawner done → init pose executor + leader bridge + rviz.
    # The bridge is started after the arm spawner (not after init pose) so
    # that the HX5 hand starts following the leader trigger immediately
    # while the arm is still moving to home. Keep the leader still until
    # the arm has finished homing.
    start_arm_followups_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[arm_init_position_executor, leader_traj_bridge, rviz_node],
        )
    )

    return [
        # Always-on nodes
        hand_robot_state_publisher,
        hand_control_node,
        arm_robot_state_publisher,
        arm_control_node,
        leader_launch,
        # Sequenced actions
        delayed_hand_spawner,
        start_hand_current_after_spawner,
        start_synctable_after_spawner,
        start_hand_initial_after_synctable,
        start_arm_after_hand_initial,
        start_arm_followups_after_spawner,
    ]
