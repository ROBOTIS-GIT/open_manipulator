from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    # Step 1: Start follower launch file
    start_follower = ExecuteProcess(
        cmd=["ros2", "launch", "open_manipulator_bringup", "hardware_y_follower.launch.py"],
        output="screen"
    )

    # Step 2: Run the initialization script for the follower
    init_follower = ExecuteProcess(
        cmd=["ros2", "run", "open_manipulator_bringup", "init_position_for_follower.py"],
        output="screen",
        shell=True
    )

    # Step 3: Start leader launch file
    start_leader = ExecuteProcess(
        cmd=["ros2", "launch", "open_manipulator_bringup", "hardware_y_leader.launch.py"],
        output="screen",
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg="🚀 Starting hardware_y_follower.launch.py..."),
        start_follower,

        # Step 2: Ensure init_follower starts only after start_follower is fully launched
        RegisterEventHandler(
            OnProcessStart(
                target_action=start_follower,
                on_start=[
                    LogInfo(msg="✅ hardware_y_follower.launch.py has fully started. Running init_position_for_follower.py..."),
                    init_follower
                ]
            )
        ),

        # Step 3: Ensure start_leader starts only after init_follower has fully started & exited
        RegisterEventHandler(
            OnProcessExit(
                target_action=init_follower,
                on_exit=[
                    LogInfo(msg="✅ init_position_for_follower.py has fully executed and exited. Starting hardware_y_leader.launch.py..."),
                    start_leader
                ]
            )
        ),
    ])
