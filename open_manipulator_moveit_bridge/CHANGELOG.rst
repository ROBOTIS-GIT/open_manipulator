^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package open_manipulator_position_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-06-01)
------------------
* package reconfiguration for OpenManipulator
* added pick and place controller
* added private node handler, init_position arg, state machine, task, joint limit param
* added scaling factor for moving time and block making another path when robot moving
* modified publisher, velocity scaling factor
* modified serive to get pose data
* deleted boost lib, unused variables
* merged pull request `#34 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/34>`_ `#33 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/33>`_ `#31 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/31>`_ `#27 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/27>`_ `#26 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/26>`_ `#25 <https://github.com/ROBOTIS-GIT/open_manipulator/issues/25>`_
* Contributors: Darby Lim, Pyo

0.1.1 (2018-03-15)
------------------
* modified build setting for using yaml-cpp
* Contributors: Pyo

0.1.0 (2018-03-14)
------------------
* modified control frequency
* modified joint control
* modified gripper topic
* modified moveit setting and gripper control
* modified gazebo sim
* modified messages and description
* refactoring for release
* modified cmake, package files for release
* Contributors: Darby Lim, Pyo
