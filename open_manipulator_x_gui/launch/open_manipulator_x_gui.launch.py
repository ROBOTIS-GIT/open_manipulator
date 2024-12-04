import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    csv_file_path = os.path.join(
        get_package_share_directory('open_manipulator_x_gui'),
        'config', 'robot_joint_log.csv')

    gui_node = Node(
        package='open_manipulator_x_gui',
        executable='open_manipulator_x_gui_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'csv_path': csv_file_path}
        ]
    )

    return LaunchDescription([
        gui_node
    ])
