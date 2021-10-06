^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package open_manipulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
