^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package open_manipulator_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.9 (2025-09-16)
------------------
* Support custom camera namespace and connection
* Contributors: Junha Cha

4.0.8 (2025-09-03)
------------------
* Added camera_usb_cam launch file
* Support OMX series
* Removed unused use_sim_time parameter in the configuration files
* Added OMY-F3M Leader and OMY-L100 Follower configuration files
* Contributors: Woojin Wie, Junha Cha

4.0.7 (2025-07-17)
------------------
* Updated launch files for OMY Packing and Unpacking
* Contributors: Woojin Wie

4.0.6 (2025-07-15)
------------------
* Renamed omx to open_manipulator_x
* Contributors: Wonho Yun

4.0.5 (2025-07-02)
------------------
* Added init_position_file argument to launch files
* Added qc_position file for OMY series
* Enabled self-collision avoidance by default for OMY series
* Contributors: Woojin Wie, Sungho Woo

4.0.4 (2025-06-26)
------------------
* None

4.0.3 (2025-06-25)
------------------
* None

4.0.2 (2025-06-25)
------------------
* Fixed launch file paths for OMY-3M and OMY-F3M
* Contributors: Woojin Wie

4.0.1 (2025-06-23)
------------------
* None

4.0.0 (2025-06-19)
------------------
* Refactored the package to support the new OMY-3M, OMY-F3M, OMY-L100
* Contributors: Woojin Wie, Wonho Yun

3.3.0 (2025-06-10)
------------------
* Added self-collision functionality to OMY Follower
* Contributors: Sungho Woo

3.2.4 (2025-05-30)
------------------
* None

3.2.3 (2025-05-07)
------------------
* Updated udev settings for improved device recognition
* Contributors: Wonho Yun

3.2.2 (2025-04-17)
------------------
* Optimized ROS2 control configurations for better performance
* Contributors: Woojin Wie

3.2.1 (2025-04-11)
------------------
* None

3.2.0 (2025-04-09)
------------------
* Updated OM-Y Follower for improved joint initialization and action-based ROS2 control
* Contributors: Woojin Wie

3.1.0 (2025-03-17)
------------------
* Integrate OM-X, OM-Y, and OM-Teleoperation with Jazzy support and Gazebo Harmonic compatibility
* Contributors: Sungho Woo

3.0.0 (2024-12-06)
------------------
* Refactored OM-X for compatibility with MoveIt 2
* Contributors: Wonho Yoon, Sungho Woo

2.3.0 (2021-10-06)
------------------
* ROS2 Foxy Fitzroy supported
* OpenMANIPULATOR Teleop developed in python
* Contributors: Will Son

2.2.0 (2019-11-13)
------------------
* Applied robotis coding style guide
* Contributors: Ryan Shim

2.1.0 (2019-08-31)
------------------
* Added support for ROS2
* Contributors: Ryan Shim

2.0.1 (2019-02-18)
------------------
* added dependency option for open_manipulator_control_gui package
* Contributors: Pyo

2.0.0 (2019-02-08)
------------------
* updated the CHANGELOG and version to release binary packages
* added new packages (open_manipulator_control_gui, *_controller, *_libs, *_teleop)
* deleted unused packages (open_manipulator_dynamixel_ctrl, open_manipulator_position_ctrl)
* - open_manipulator_control_gui -
* updated function name, UI
* added group names and gripper args
* added position only client
* modified topic names, end-effector name
* - open_manipulator_controller -
* added jointspace path serv, moveit params
* added moveit config and controller
* added kinematic pose pub
* added mimic param and end effector point
* added execute permission
* added usb rules
* added cdc rules
* removed warn message
* renamed open_manipulator lib files
* changed math function name, namespace
* changed openManipulatorProcess() to processOpenManipulator()
* updated start_state after execution on MoveIt
* updated thread time, dynamixel profiling control method
* updated drawing line
* updated flexible node
* updated tool control
* updated chain to open_manipulator
* updated new kinematics
* used robot_name on joint_state_publisher's source_list
* - open_manipulator_description -
* deleted model.launch
* modified gripper origin
* modified end_effector origin
* modified link2 and joint2 position
* updated inertia
* changed calculated inertia param
* changed gripper link name
* changed axis for grip_joint
* - open_manipulator_moveit -
* added moveit config and controller
* updated moveit rviz
* Updated start_state after execution on Moveit `#83 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/83>`_
* changed control period 40mm to 100mm
* Contributors: Darby Lim, Hye-Jong KIM, Yong-Ho Na, Ryan Shim, Guilherme de Campos Affonso, Pyo

1.0.0 (2018-06-01)
------------------
* package reconfiguration for OpenManipulator
* added new stl files
* added urdf, rviz param, gazebo params, group
* added function to support protocol 1.0
* modified color, xacro server, mu1, mu2, collision range, joint limit
* modified joint_state_publisher, joint_states_publisher
* modified params of inertial, xacro, gazebo, collision, friction
* modified urdf file names and collision geometry
* modified motor id, msg names
* modified description and package tree
* deleted unnecessary packages
* merged pull request `#34 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/34>`_ `#33 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/33>`_ `#32 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/32>`_ `#31 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/31>`_ `#27 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/27>`_ `#26 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/26>`_ `#25 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/25>`_
* Contributors: Darby Lim, Pyo

0.1.1 (2018-03-15)
------------------
* modified build setting for using yaml-cpp
* Contributors: Pyo

0.1.0 (2018-03-14)
------------------
* added meta package for OpenManipulator
* updated dynamixel controller
* modified joint control
* modified gripper topic
* modified URDF
* modified description
* modified messages
* modified moveit set and gripper control
* modified gazebo and moveit setting
* modified cmake, package files for release
* refactoring for release
* Contributors: Darby Lim, Pyo
