^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package om_gravity_compensation_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.9 (2025-09-15)
------------------
* None

4.0.8 (2025-09-03)
------------------
* Added parameter for enabling spring effect
* Added parameters about scaling factors for input joint velocities and accelerations
* Contributors: Woojin Wie

4.0.7 (2025-07-17)
------------------
* None

4.0.6 (2025-07-15)
------------------
* None

4.0.5 (2025-07-02)
------------------
* Added feedback control for leader-follower synchronization
* Contributors: Sungho Woo

4.0.4 (2025-06-26)
------------------
* None

4.0.3 (2025-06-25)
------------------
* None

4.0.2 (2025-06-25)
------------------
* None

4.0.1 (2025-06-23)
------------------
* None

4.0.0 (2025-06-19)
------------------
* Fix the velocity unit issue to match the new dynamixel_hardware_interface version
* Contributors: Woojin Wie

3.3.0 (2025-06-10)
------------------
* None

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
* None

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
