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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction #new
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
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake sensor commands.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='true',
            description='Whether to launch the init_position node',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='open_manipulator_x',
            description='Type of ros2_control',
        ),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='initial_positions.yaml',
            description='Path to the initial position file',
        ),
    ]

    # Launch configurations
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')

    # Generate URDF file using xacro
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

    # Paths for configuration files
    controller_manager_config = '/root/colcon_ws/src/open_manipulator/open_manipulator_bringup/config/omx_leader/hardware_controller_manager.yaml'
    #PathJoinSubstitution([
    #    FindPackageShare('open_manipulator_bringup'),
    #    'config',
    #    'open_manipulator_x',
    #    'hardware_controller_manager.yaml',
    #])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator.rviz',
    ])

    trajectory_params_file = '/root/colcon_ws/src/open_manipulator/open_manipulator_bringup/config/omx_leader/initial_positions.yaml'
    #PathJoinSubstitution([
    #    FindPackageShare('open_manipulator_bringup'),
    #    'config',
    #    'open_manipulator_x',
    #    init_position_file,
    #])

    # Define nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    # 컨트롤러 스폰 순서: JS Broadcaster → Arm/Gripper
    # ① Joint State Broadcaster 스폰
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/leader/controller_manager'
        ],
        namespace='leader',  # 네임스페이스 분리로 다중 로봇 충돌 방지
        output='screen',
    )

    # ② Arm + Gripper 컨트롤러 스폰 (JSB 종료 후 실행되도록 이벤트 핸들러 설정)
    arm_grip_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            '--controller-manager', '/leader/controller_manager'
        ],
        namespace='leader',
        output='screen',
    )

    # JSB 종료(on_exit) 시 Arm/Gripper 스폰
    robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_grip_spawner],
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim, 'frame_prefix': 'leader_'}],
        output='both',
    )

    joint_trajectory_executor = Node(
    
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[
        trajectory_params_file,
        { 'action_topic': '/leader/arm_controller/follow_joint_trajectory',
            'joint_states_topic': '/leader/joint_states' }  
        ],
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
    #토크 0 만들기 명령 정의
    torque_off_cmd = ExecuteProcess( 
        cmd=[
        'ros2', 'service', 'call',
        '/leader/dynamixel_hardware_interface/set_dxl_torque',
        'std_srvs/srv/SetBool',
        "{data: false}"
        ],
        output='screen'
    )

   

    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_grip_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )
    #initial position 이후 1초 대기 후 토크 0 만들기
    torque_off_timer = TimerAction(period=1.0, actions=[torque_off_cmd])

    disable_torque_after_init = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_executor,
            on_exit=[torque_off_timer],
        )
    )

    leader_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            control_node,
            jsb_spawner,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_joint_trajectory_executor_after_controllers,
            disable_torque_after_init 
            
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
        
            leader_with_namespace
        ]
    )
