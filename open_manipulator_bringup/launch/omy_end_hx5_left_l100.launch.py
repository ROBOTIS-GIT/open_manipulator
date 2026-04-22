#!/usr/bin/env python3
"""
OMY END + HX5 left hand follower with L100 leader.

Follower side has NO arm — only the HX5 left hand via OMY END SyncTable relay.
L100 leader publishes arm+gripper trajectories; the trajectory bridge extracts
the gripper value and maps it to HX5 finger presets.

Usage:
  ros2 launch open_manipulator_bringup omy_end_hx5_left_l100.launch.py hand_port_name:=/dev/ttyUSB1 leader_port_name:=/dev/ttyUSB2
"""

import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetRemap

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'hand_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for OMY END (HX5 hand).',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for HX5 hand.',
        ),
        DeclareLaunchArgument(
            'leader_port_name',
            default_value='/dev/ttyUSB1',
            description='Serial port for OMY L100 leader.',
        ),
        DeclareLaunchArgument(
            'use_self_collision_avoidance',
            default_value='true',
            description='Enable self-collision detection on the leader.',
        ),
        DeclareLaunchArgument(
            'leader_use_sim',
            default_value='false',
            description='If true, pass use_sim:=true to leader launch.',
        ),
    ]

    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context):
    hand_port_name = LaunchConfiguration('hand_port_name').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    leader_port_name = LaunchConfiguration('leader_port_name')
    use_self_collision_avoidance = LaunchConfiguration('use_self_collision_avoidance')
    leader_use_sim = LaunchConfiguration('leader_use_sim')

    # ── Build URDF at launch time (avoids DDS stale-topic race) ──────────
    xacro_file = os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'urdf', 'omy_f3m', 'omy_end_hx5_left_standalone.urdf.xacro')
    urdf_str = subprocess.check_output([
        'xacro', xacro_file,
        f'port_name:={hand_port_name}',
        f'use_mock_hardware:={use_mock_hardware}',
    ]).decode('utf-8')

    controller_manager_config = os.path.join(
        get_package_share_directory('open_manipulator_bringup'),
        'config', 'omy_end_hx5_left_l100',
        'hardware_controller_manager.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_str}],
        remappings=[
            ('robot_description', '/hand_robot_description'),
            ('joint_states', '/hand/joint_states'),
        ],
        output='both',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='hand',
        parameters=[{'robot_description': urdf_str}, controller_manager_config],
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

    left_current_command_process = ExecuteProcess(
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

    initial_pose_process = ExecuteProcess(
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

    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_f3m_hx5_left_leader_trajectory_bridge',
        parameters=[
            {'leader_trajectory_topic': '/leader/joint_trajectory'},
            {'hand_command_topic': '/hand/hand_l_controller/joint_trajectory'},
        ],
        output='both',
    )

    # ── L100 leader ──────────────────────────────────────────────────────
    # Wrap in GroupAction + SetRemap to avoid DDS stale robot_description
    # messages from previous runs causing the leader to init with wrong port.
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
            ],
        ),
    ])

    # ── Sequencing ───────────────────────────────────────────────────────
    # robot_description is now passed as a concrete string parameter,
    # so the controller_manager uses it directly (no topic race / segfault).

    # Step 1: 5s after launch → spawn controllers
    delayed_spawner = TimerAction(
        period=5.0,
        actions=[hand_controller_spawner],
    )

    # Step 2: after spawner completes → start effort command (Goal Current=300)
    start_effort_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[left_current_command_process],
        )
    )

    # Step 3: after spawner completes → enable SyncTable (with 3s internal delay
    #         so BulkWrite already contains Goal Current=300 before relay starts)
    start_synctable_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[enable_synctable_process],
        )
    )

    # Step 4: after SyncTable enabled → send initial pose (2s delay)
    delayed_initial_pose = TimerAction(
        period=2.0,
        actions=[initial_pose_process],
    )
    start_initial_pose_after_synctable = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=enable_synctable_process,
            on_exit=[delayed_initial_pose],
        )
    )

    # Step 5: after initial pose → start trajectory bridge
    start_traj_bridge_after_pose = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_pose_process,
            on_exit=[leader_traj_bridge],
        )
    )

    return [
        robot_state_publisher_node,
        control_node,
        delayed_spawner,
        leader_launch,
        start_effort_after_spawner,
        start_synctable_after_spawner,
        start_initial_pose_after_synctable,
        start_traj_bridge_after_pose,
    ]
