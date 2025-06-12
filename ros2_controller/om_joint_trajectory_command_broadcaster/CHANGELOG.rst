^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package om_joint_trajectory_command_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.0 (2025-06-10)
------------------
* Added self-collision functionality to OMY Follower
* Contributors: Sungho Woo

3.2.4 (2025-05-30)
------------------
* Modified ROS2 controller package dependencies
* Fixed stderr output handling
* Deprecate ament_include_dependency usage in CMakeLists.txt
* Contributors: Wonho Yun

3.2.3 (2025-05-07)
------------------
* None

3.2.2 (2025-04-17)
------------------
* Handle lint errors
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
* Compensated for gravity in a robot arm. It uses the KDL library to compute the torques required to maintain the arm in a desired position.
* Contributors: Woojin Wie
