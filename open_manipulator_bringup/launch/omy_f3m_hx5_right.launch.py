#!/usr/bin/env python3
"""
omy_f3m_hx5_right.launch.py

Combined teleoperation launch:
  * F3M follower arm (joint1-6)             ← driven by filtered leader trajectory
  * HX5 right hand via OMY END SyncTable    ← driven by leader_trajectory_bridge
  * L100 leader (publishes /leader/...)

Topology (defaults match omy-SNPR44B9041 baseboard)
  /dev/ttyAMA2 : F3M follower arm (built-in default in omy_f3m_position xacro)
  /dev/ttyAMA4 : OMY END (HX5 hand, hub ID 210, Tool Bus to HX5 motors 111..135)
  /dev/ttyUSB0      : L100 leader (USB-RS485 adapter)

Sequencing (mirror omy_f3m.launch.py: omy_f3m_position → omy_f3m_end_unit)
  In the upstream omy_f3m.urdf.xacro both hardware systems live in the same
  controller_manager process and are initialised in xacro order:
      omy_f3m_position (arm, /dev/ttyAMA2 @ 6.25 Mbps) FIRST
      omy_f3m_end_unit  (P12 + OMY END, /dev/ttyAMA4 @ 4 Mbps) SECOND
  By the time the second hardware system opens its UART, the kernel /
  RP1 driver / process scheduler is already warm from the first system's
  ping+InitItem traffic. We mirror that by:

      t = 0      arm_robot_state_publisher
                 hand_robot_state_publisher  (passive — just /tf)
                 arm_control_node            ← omy_f3m_position equivalent
      t = 10 s   hand_control_node           ← omy_f3m_end_unit equivalent
                 (cold-port ping to ID 210 happens AFTER the arm has
                  finished InitDxlComm to IDs 1-6 + omy_hat, so the
                  RP1 UART subsystem is no longer in a fresh-boot state)

  During hand_control_node on_init the normal xacro InitItem chain runs:
      Step 1: omy_end_disable_synctable + omy_end_pre
                              (SyncTable Enable=0, then Tool Bus cfg)
      Step 2: hand_r_controller
                              (ID110 Table Sync config, Table Sync Enable=0)
      Step 3: dxl111..dxl134 motor init
                              (Operating Mode, gains, indirect addresses)
      Step 4: hand_r_controller_dummy
                              (ID110 Table Sync Enable=1)
      Step 5: virtual_dxl mapping through OMY END ID210
      Step 6: omy_end_init       (placeholder)
      Step 7: launch-time service calls
                              (SyncTable Enable=1, SyncTable Enable HX5=1)

  configure_hx5_synctable:=true adds the persistent HX5 ID110 SyncTable
  EEPROM setup and per-motor init writes. Use it only for one-time setup,
  not for normal launches.

      t = 18 s   hand_controller_spawner     (≈8 s into hand init = safe)
      hand spawner exits → hand current cmd + (2 s later) hand initial pose
      hand initial pose exits → arm_controller_spawner
      arm spawner exits → joint_trajectory_executor + rviz
      follower home pose exits → leader_launch
      leader start + 2 s → leader bridge

Usage
  ros2 launch open_manipulator_bringup omy_f3m_hx5_right.launch.py \
      use_self_collision_avoidance:=false
  # Override only when a port differs from the baseboard defaults:
  #   hand_port_name:=/dev/ttyAMA4   leader_port_name:=/dev/ttyUSB0
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'hand_port_name',
            default_value='auto',
            description='Serial port for OMY END (HX5 hand, hub ID 210). '
                        'Use "auto" to prefer /dev/ttyAMA4, then /dev/rp1ctrluart4.',
        ),
        DeclareLaunchArgument(
            'leader_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for OMY L100 leader (USB-RS485 adapter).',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware for both HX5 and follower arm.',
        ),
        DeclareLaunchArgument(
            'configure_hx5_synctable',
            default_value='false',
            description='Write persistent HX5 ID110 SyncTable EEPROM setup during hand init. '
                        'Use only for one-time configuration/debugging; normal launches skip it.',
        ),
        DeclareLaunchArgument(
            'enable_hx5_motor_torque',
            default_value='false',
            description='Deprecated debug path for direct HX5 motor InitItem writes. '
                        'Keep false for OMY END SyncTable operation.',
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
        DeclareLaunchArgument(
            'enable_synctable_watchdog',
            default_value='false',
            description='Read back and re-enable OMY END SyncTable periodically. '
                        'Keep false while validating the HX5 relay because service reads can stall the hand bus.',
        ),
    ]
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=_launch_setup),
    ])


def _resolve_hand_port(requested_port: str) -> str:
    if requested_port and requested_port != 'auto':
        if not os.path.exists(requested_port):
            raise RuntimeError(
                f'OMY END hand_port_name "{requested_port}" does not exist. '
                'Use hand_port_name:=/dev/ttyAMA4, or check ls -l /dev/ttyAMA* /dev/rp1ctrluart*.'
            )
        return requested_port

    for candidate in ('/dev/ttyAMA4', '/dev/rp1ctrluart4'):
        if os.path.exists(candidate):
            return candidate

    raise RuntimeError(
        'Could not find an OMY END serial port. Expected /dev/ttyAMA4 or '
        '/dev/rp1ctrluart4. Check device names with: ls -l /dev/ttyAMA* /dev/rp1ctrluart*'
    )


def _launch_setup(context):
    hand_port_name = _resolve_hand_port(
        LaunchConfiguration('hand_port_name').perform(context)
    )
    leader_port_name = LaunchConfiguration('leader_port_name')
    use_mock_hardware_str = LaunchConfiguration('use_mock_hardware').perform(context)
    configure_hx5_synctable_str = LaunchConfiguration('configure_hx5_synctable').perform(context)
    enable_hx5_motor_torque_str = LaunchConfiguration('enable_hx5_motor_torque').perform(context)
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
    enable_synctable_watchdog = LaunchConfiguration('enable_synctable_watchdog')

    # ─── HX5 hand subsystem (namespace /hand) ───────────────────────────
    # URDF is materialized at launch time so the controller_manager gets
    # robot_description as a concrete string parameter (avoids DDS races
    # against stale topic publishers from previous runs).
    hand_xacro = os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'urdf', 'omy_f3m', 'omy_end_hx5_right_standalone.urdf.xacro')
    # dynamixel_hardware_interface prepends its own package share directory to
    # this parameter at runtime (see dynamixel_hardware_interface.cpp line ~144).
    # Compute the relative path from that share dir to the actual model folder
    # so the concatenation resolves correctly regardless of install prefix.
    import os as _os
    _dxl_share = get_package_share_directory('dynamixel_hardware_interface')
    _model_abs = _os.path.join(
        get_package_share_directory('open_manipulator_description'),
        'param', 'dxl_model_omy_end_hx5_right')
    hand_dynamixel_model_folder = '/' + _os.path.relpath(_model_abs, _dxl_share)
    hand_urdf_str = subprocess.check_output([
        'xacro', hand_xacro,
        f'port_name:={hand_port_name}',
        f'dynamixel_model_folder:={hand_dynamixel_model_folder}',
        f'use_mock_hardware:={use_mock_hardware_str}',
        f'configure_hx5_synctable:={configure_hx5_synctable_str}',
        f'enable_hx5_motor_torque:={enable_hx5_motor_torque_str}',
    ]).decode('utf-8')

    hand_controller_manager_config = os.path.join(
        get_package_share_directory('open_manipulator_bringup'),
        'config', 'omy_end_hx5_right_l100',
        'hardware_controller_manager.yaml')

    hand_robot_description_param = ParameterValue(hand_urdf_str, value_type=str)
    hand_robot_description_topic = '/hand_robot_description_hx5_right'

    hand_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': hand_robot_description_param}],
        remappings=[
            ('robot_description', hand_robot_description_topic),
            ('joint_states', '/hand/joint_states'),
        ],
        output='both',
    )

    hand_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='hand',
        parameters=[
            {'robot_description': hand_robot_description_param},
            hand_controller_manager_config,
        ],
        remappings=[('robot_description', hand_robot_description_topic)],
        output='both',
        condition=UnlessCondition(leader_use_sim),
    )

    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'hand_r_controller',
            'effort_r_controller',
            '-c', '/hand/controller_manager',
        ],
        output='both',
    )

    hand_current_command_process = ExecuteProcess(
        name='hand_r_current_command',
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '10',
            '/hand/effort_r_controller/commands',
            'std_msgs/msg/Float64MultiArray',
            'data: [300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0,'
                    '300.0, 300.0, 300.0, 300.0]',
        ],
    )

    hand_initial_pose_process = ExecuteProcess(
        name='hand_r_initial_pose',
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/hand/hand_r_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{'
            'joint_names: ['
            'finger_r_joint1, finger_r_joint2, finger_r_joint3, finger_r_joint4, '
            'finger_r_joint5, finger_r_joint6, finger_r_joint7, finger_r_joint8, '
            'finger_r_joint9, finger_r_joint10, finger_r_joint11, finger_r_joint12, '
            'finger_r_joint13, finger_r_joint14, finger_r_joint15, finger_r_joint16, '
            'finger_r_joint17, finger_r_joint18, finger_r_joint19, finger_r_joint20], '
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
    # Mirrors omy_f3m.launch.py but:
    #   (a) leaves /arm_controller/joint_trajectory on its normal topic; the
    #       leader bridge filters /leader/joint_trajectory down to joint1..6,
    #   (b) does NOT spawn gripper_controller (the leader trigger drives HX5
    #       via the bridge instead),
    #   (c) uses omy_f3m_arm_only.urdf.xacro instead of omy_f3m.urdf.xacro.
    #
    # Why (c): the standard omy_f3m.urdf.xacro always declares the legacy
    # OMYF3MEndUnitSystem (P12 gripper, OMY END on /dev/ttyAMA4 ID 210). The
    # HX5 hand controller_manager (in /hand) already owns OMY END through
    # OMYF3MEndUnitHX5RightSystem, so loading the legacy end-unit here would
    # make two ros2_control hardware components fight for the same physical
    # OMY END device, producing endless [ID:210] COMM_ERROR / BulkRead Rx
    # Fail timeouts on both sides.
    arm_urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf', 'omy_f3m', 'omy_f3m_arm_only.urdf.xacro',
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

    arm_robot_description_param = ParameterValue(arm_urdf_file, value_type=str)

    arm_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': arm_robot_description_param,
                     'use_sim_time': use_sim}],
        output='both',
    )

    arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': arm_robot_description_param},
                    arm_controller_manager_config],
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
        parameters=[{'robot_description': arm_robot_description_param}],
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
                ('use_mock_hardware', use_mock_hardware),
                # Override the parent launch's ros2_control_type
                # (which is omy_f3m_position for the follower arm) so the
                # leader xacro picks up its own omy_l100_current variant
                # and finds xacro:omy_l100_system.
                ('ros2_control_type', 'omy_l100_current'),
            ],
        ),
    ])

    # ─── SyncTable watchdog ──────────────────────────────────────────────
    # OMY END firmware can spontaneously drop SyncTable Enable / SyncTable
    # Enable HX5 to 0 under heavy bus contention. Without this watchdog
    # the HX5 hand stops responding any time after launch.
    hand_synctable_watchdog = Node(
        package='open_manipulator_bringup',
        executable='synctable_watchdog',
        name='hand_synctable_watchdog',
        parameters=[
            {'hub_id': 210},
            {'items': ['SyncTable Enable', 'SyncTable Enable HX5']},
            {'check_period_sec': 1.0},
            {'set_service': '/hand/dynamixel_hardware_interface/set_dxl_data'},
            {'get_service': '/hand/dynamixel_hardware_interface/get_dxl_data'},
        ],
        output='both',
        condition=IfCondition(enable_synctable_watchdog),
    )

    # ─── Bridge: leader arm joints → F3M, rh_r1_joint → HX5 presets ─────
    leader_traj_bridge = Node(
        package='open_manipulator_bringup',
        executable='omy_f3m_hx5_right_leader_trajectory_bridge',
        parameters=[
            {'leader_trajectory_topic': '/leader/joint_trajectory'},
            {'arm_command_topic': '/arm_controller/joint_trajectory'},
            {'hand_command_topic': '/hand/hand_r_controller/joint_trajectory'},
        ],
        output='both',
    )

    # ─── Sequencing ─────────────────────────────────────────────────────
    # ORDER REVERSAL (vs previous revision):
    #   Mirror omy_f3m.launch.py where omy_f3m_position is initialised
    #   BEFORE omy_f3m_end_unit. Empirically the cold-port ping to OMY END
    #   (ID 210) at 4 Mbps right after openPort + setBaudRate hits a
    #   stack-buffer overflow in dynamixel_sdk Protocol2PacketHandler::
    #   rxPacket() when residual / self-echo bytes form a "fake header"
    #   that bumps wait_length past the 11/14-byte rxpacket buffer.
    #
    #   Letting the F3M arm finish its own InitDxlComm sweep on
    #   /dev/ttyAMA2 first gives the RP1 UART subsystem ~10 s of
    #   warm-up time (kernel scheduler, RP1 firmware DMA primed,
    #   ros2_control library code paths JIT-loaded) before the hand
    #   process opens the OMY END port and pings ID 210.
    #
    # SyncTable activation itself is still done by the xacro InitItem chain
    # (Steps 1-7) inside the hand controller_manager, so no external
    # set_dxl_data service call is required.

    # Step 1: t = 10 s → hand_control_node (cold-port ping happens here).
    # The arm + leader have been running for 10 s on their own ports
    # (/dev/ttyAMA2) so the RP1 UART driver is warm and DDS
    # discovery has settled before we trigger the OMY END ping.
    delayed_hand_control_node = TimerAction(
        period=10.0,
        actions=[hand_control_node],
    )

    # Step 2: t = 18 s → spawn HX5 controllers. The hand's full InitItem
    # chain (OMY END pre + HX5 hub SyncTable cfg + 24 motor inits + OMY
    # END post) takes ~5-8 s, so 8 s after hand_control_node start is the
    # safe spawn window.
    delayed_hand_spawner = TimerAction(
        period=18.0,
        actions=[hand_controller_spawner],
    )

    # Step 2a: HX5 spawner done → start the constant Goal Current=300
    # publisher so the BulkWrite buffer carries non-zero current.
    start_hand_current_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[hand_current_command_process],
        )
    )

    # Step 2b: HX5 spawner done → 2 s later send HX5 initial pose.
    delayed_hand_initial_pose = TimerAction(
        period=2.0,
        actions=[hand_initial_pose_process],
    )
    start_hand_initial_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[delayed_hand_initial_pose],
        )
    )

    # Step 2c: Enable OMY END SyncTable only after Goal Current and the
    # initial pose have been written into the BulkWrite buffer. HX5 hub
    # Table Sync is configured during xacro hardware init; do not call ID 110
    # directly here because it is behind the OMY END tool bus.
    hand_enable_omy_sync = ExecuteProcess(
        name='hand_enable_omy_sync',
        cmd=[
            'ros2', 'service', 'call', '/hand/dynamixel_hardware_interface/set_dxl_data',
            'dynamixel_interfaces/srv/SetDataToDxl', '{id: 210, item_name: "SyncTable Enable", item_data: 1}'
        ],
        output='both',
    )

    hand_enable_omy_hx5_sync = ExecuteProcess(
        name='hand_enable_omy_hx5_sync',
        cmd=[
            'ros2', 'service', 'call', '/hand/dynamixel_hardware_interface/set_dxl_data',
            'dynamixel_interfaces/srv/SetDataToDxl', '{id: 210, item_name: "SyncTable Enable HX5", item_data: 1}'
        ],
        output='both',
    )

    start_hand_services_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_controller_spawner,
            on_exit=[
                TimerAction(period=3.0, actions=[hand_enable_omy_sync]),
                TimerAction(period=4.0, actions=[hand_enable_omy_hx5_sync]),
            ],
        )
    )

    # Step 3: OMY END SyncTable enabled → spawn follower arm controllers.
    # The watchdog is optional because its service reads share the same DXL
    # port and can stall a slow OMY END SyncTable read/write cycle.
    start_arm_after_hand_sync = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=hand_enable_omy_hx5_sync,
            on_exit=[hand_synctable_watchdog, arm_controller_spawner],
        )
    )

    # Step 4: arm spawner done → init pose executor + leader bridge + rviz.
    # Keep the leader and bridge stopped here so the follower reaches its
    # configured home pose before any leader trajectory can command motion.
    start_arm_followups_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[arm_init_position_executor, rviz_node],
        )
    )

    # Step 5: follower home pose done → start leader, then let it publish an
    # initial position before enabling the leader-to-HX5/arm bridge.
    delayed_leader_traj_bridge = TimerAction(
        period=2.0,
        actions=[leader_traj_bridge],
    )
    start_leader_after_home = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_init_position_executor,
            on_exit=[leader_launch, delayed_leader_traj_bridge],
        ),
        condition=IfCondition(init_position),
    )

    delayed_leader_traj_bridge_without_home = TimerAction(
        period=2.0,
        actions=[leader_traj_bridge],
    )
    start_leader_without_home = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[leader_launch, delayed_leader_traj_bridge_without_home],
        ),
        condition=UnlessCondition(init_position),
    )

    return [
        LogInfo(msg=f'Using OMY END hand port: {hand_port_name}'),
        # Always-on passive nodes (just /tf, no hardware traffic)
        hand_robot_state_publisher,
        arm_robot_state_publisher,
        # t = 0 : arm hardware comes up first (omy_f3m_position equivalent).
        #         Its UART traffic warms the RP1 driver before OMY END opens.
        arm_control_node,
        # t = 10 s : hand hardware comes up (omy_f3m_end_unit equivalent).
        delayed_hand_control_node,
        # t = 18 s : spawn HX5 controllers, then chain the rest.
        delayed_hand_spawner,
        start_hand_current_after_spawner,
        start_hand_initial_after_spawner,
        start_hand_services_after_spawner,
        start_arm_after_hand_sync,
        start_arm_followups_after_spawner,
        start_leader_after_home,
        start_leader_without_home,
    ]
