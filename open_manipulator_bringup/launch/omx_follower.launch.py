#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Sungho Woo, Woojin Wie

# -----------------------------------------------------------------------------
# 한국어 설명
# -----------------------------------------------------------------------------
# 이 런치 파일은 ROS 2 기반 OpenMANIPULATOR‑X 로봇 팔을 실환경(하드웨어) 또는 Gazebo
# 시뮬레이터에서 구동하기 위한 전체 bring‑up 절차를 자동화합니다. RViz 시각화, 컨트롤러
# 스폰, 초기 자세 지정, 네임스페이스 분리 등을 한 번에 실행할 수 있도록 구성되어 있어
# 별도의 명령 입력 없이 로봇 시스템을 손쉽게 기동할 수 있습니다.
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction


def generate_launch_description():
    # --------------------------- 런치 인자 선언 ---------------------------
    # 아래 항목들은 런치 실행 시 사용자가 "ros2 launch ..." 명령에서
    # "name:=value" 형태로 덮어쓸 수 있는 옵션들입니다.
    declared_arguments = [
        # RViz 시각화 노드 실행 여부
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        # 조인트 및 링크 네이밍 프리픽스 (다중 로봇 구동 시 유용)
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        # Gazebo 시뮬레이터 사용 여부
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        # Fake Hardware 기능 사용 여부 (실제 하드웨어 없이 ros2_control 인터페이스 테스트)
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware mirroring command.',
        ),
        # Fake 센서 명령 발행 여부
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake sensor commands.',
        ),
        # 실하드웨어 연결 시 통신 포트 경로
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ),
        # 초기 자세(joint trajectory) 노드 실행 여부
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Whether to launch the init_position node',
        ),
        # ros2_control 플러그인 타입 지정
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='open_manipulator_x',
            description='Type of ros2_control',
        ),
        # 초기 자세에 사용할 YAML 파일명
        DeclareLaunchArgument(
            'init_position_file',
            default_value='initial_positions.yaml',
            description='Path to the initial position file',
        ),
    ]

    # --------------------------- 런치 구성 변수 ---------------------------
    # LaunchConfiguration 객체를 통해 런치 인자 값을 Python 변수처럼 사용합니다.
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    # --------------------------- URDF(xacro) 생성 ---------------------------
    # xacro 파일에 런치 인자 값을 전달하여 최종 URDF 문자열을 생성합니다.
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf',
            'open_manipulator_x',
            'open_manipulator_x.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=',
        use_sim,
        ' ',
        'use_fake_hardware:=',
        use_fake_hardware,
        ' ',
        'fake_sensor_commands:=',
        fake_sensor_commands,
        ' ',
        'port_name:=',
        port_name,
        ' ',
        'ros2_control_type:=',
        ros2_control_type,
    ])

    # --------------------------- 설정 파일 경로 ---------------------------
    # 아래 경로는 Docker/colcon 워크스페이스 내 상대 경로로 하드코딩 되어 있습니다.
    # 필요 시 PathJoinSubstitution을 사용하여 패키지 설치 위치를 동적으로 찾아올 수 있습니다.
    controller_manager_config = '/root/colcon_ws/src/open_manipulator/open_manipulator_bringup/config/omx_follower/hardware_controller_manager.yaml'
    # controller_manager_config = PathJoinSubstitution([
    #     FindPackageShare('open_manipulator_bringup'),
    #     'config',
    #     'omx_follower',
    #     'hardware_controller_manager.yaml',
    # ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    trajectory_params_file = '/root/colcon_ws/src/open_manipulator/open_manipulator_bringup/config/omx_follower/initial_positions.yaml'
    # trajectory_params_file = PathJoinSubstitution([
    #     FindPackageShare('open_manipulator_bringup'),
    #     'config',
    #     'omx_follower',
    #     init_position_file,
    # ])

    # --------------------------- 노드 정의 ---------------------------
    # ros2_control 노드: 하드웨어 인터페이스 + 컨트롤러 매니저 구동
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),  # 시뮬레이션 모드가 아닐 때만 실행
        # remappings=[('/arm_controller/joint_trajectory', '/leader/joint_trajectory')],
    )

    # 컨트롤러 스폰 순서: JS Broadcaster → Arm/Gripper
    # ① Joint State Broadcaster 스폰
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/follower/controller_manager'
        ],
        namespace='follower',  # 네임스페이스 분리로 다중 로봇 충돌 방지
        output='screen',
    )

    # ② Arm + Gripper 컨트롤러 스폰 (JSB 종료 후 실행되도록 이벤트 핸들러 설정)
    arm_grip_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            '--controller-manager', '/follower/controller_manager'
        ],
        namespace='follower',
        output='screen',
    )

    # JSB 종료(on_exit) 시 Arm/Gripper 스폰
    robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_grip_spawner],
        )
    )

    # 로봇 상태 퍼블리셔: TF, joint_states → /tf, /follower/joint_states 퍼블리시
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim, 'frame_prefix': 'follower_'}],
        output='both',
    )

    # 초기 자세(trajectory) 실행 노드: 컨트롤러가 준비된 뒤 실행되도록 이벤트 체인 연결
    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[
        trajectory_params_file,
        { 
            'action_topic': '/follower/arm_controller/follow_joint_trajectory',
            'joint_states_topic': '/follower/joint_states'  } 
        ],
        output='both',
        condition=IfCondition(init_position),
    )

    # RViz 시각화 노드 (옵션)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='both',
        condition=IfCondition(start_rviz),
    )

    # 컨트롤러 스폰 완료 후 Trajectory Executor 실행
    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_grip_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )

    # --------------------------- 네임스페이스 그룹핑 ---------------------------
    # "follower" 네임스페이스 안에 모든 노드를 그룹핑하여 다중 로봇 사용 시 충돌 방지
    follower_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('follower'),
            control_node,
            jsb_spawner,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_joint_trajectory_executor_after_controllers,
            # delay_rviz_after_joint_state_broadcaster_spawner
        ]
    )

    # --------------------------- LaunchDescription 반환 ---------------------------
    return LaunchDescription(
        declared_arguments
        + [
            follower_with_namespace
        ]
    )
