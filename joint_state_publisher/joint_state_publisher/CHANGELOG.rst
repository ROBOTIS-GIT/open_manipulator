^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.13 (2018-03-19)
--------------------
* add bugtracker link now that this is not hosted on robot_model anymore
* Added a scrollarea around the gridlayout to support large number of joints
* pass robot objects into init_collada() and init_urdf()
* add test for collada supports
* add support for collada model : moved from https://github.com/ros/robot_model/pull/97
* Contributors: Guillaume Walck, Kei Okada, Mikael Arguedas

1.12.12 (2017-09-15)
--------------------
* Don't publish joint states if there are no joints (`#212 <https://github.com/ros/robot_model/issues/212>`_)
* Contributors: Chris Lalancette

1.12.11 (2017-06-27)
--------------------

1.12.10 (2017-06-24)
--------------------

1.12.9 (2017-04-26)
-------------------

1.12.8 (2017-03-27)
-------------------
* [joint_state_publisher] Handle time moving backwards
  Without this patch, joint_state_publisher dies whenever the ROS time moves backwards (e.g., when running `rosbag play --clock --loop`).
* Switch a couple more packages over to Chris and Shane.
* Fix rostest dependency.
* Add recursive mimic joint (`#177 <https://github.com/ros/robot_model/issues/177>`_)
  * Add recursive mimic joint
* Contributors: Alessandro Tondo, Chris Lalancette, Martin Günther, Mike Purvis

1.12.7 (2017-01-26)
-------------------
* Fixed a crash which happened when there were ``0`` free joints, opens empty window (`#178 <https://github.com/ros/robot_model/issues/178>`_)
* Contributors: Bence Magyar

1.12.6 (2017-01-04)
-------------------
* Migrated slots in joint state publisher gui to Qt5 (`#147 <https://github.com/ros/robot_model/issues/147>`_)
* Now uses GridLayout to support large numbers of joints and small screens (`#150 <https://github.com/ros/robot_model/issues/150>`_)
* Contributors: Bence Magyar, Michał Barciś

1.12.5 (2016-10-27)
-------------------
* Fix initial position of sliders in joint_state_publisher GUI (`#148 <https://github.com/ros/robot_model/issues/148>`_)
  Caused by a regression in 8c6cf9841cb, the slider positions are not initialized correctly
  from the provided zero positions at startup.
  This commit fixes the issue, by adding the call to center() again that got lost.
* Contributors: Timm Linder

1.12.4 (2016-08-23)
-------------------

1.12.3 (2016-06-10)
-------------------
* Fix circular logic in joint state publisher events (`#140 <https://github.com/ros/robot_model/issues/140>`_)
* Use signal and sys.exit to fix shutdown in joint_state_publisher (`#139 <https://github.com/ros/robot_model/issues/139>`_)
* joint_state_publisher: Change slider update method (`#135 <https://github.com/ros/robot_model/issues/135>`_)
* Contributors: Jackie Kay, vincentrou

1.12.2 (2016-04-12)
-------------------
* Migrate qt (`#128 <https://github.com/ros/robot_model/issues/128>`_)
  * Migrate JointStatePublisher from wxPython to qt5
* Contributors: Jackie Kay

1.12.1 (2016-04-10)
-------------------

1.11.8 (2015-09-11)
-------------------

1.11.7 (2015-04-22)
-------------------
* Added a randomize button for the joints.
* Contributors: Aaron Blasdel

1.11.6 (2014-11-30)
-------------------
* Added floating joints to joint types ignored by publisher
* warn when joints have no limits
* add queue_size for publisher
* Contributors: Jihoon Lee, Michael Ferguson, Shaun Edwards

1.11.5 (2014-07-24)
-------------------

1.11.4 (2014-07-07)
-------------------
* Update package.xml
  Updating author and maintainer email for consistency
* Contributors: David Lu!!

1.11.3 (2014-06-24)
-------------------

1.11.2 (2014-03-22)
-------------------

1.11.1 (2014-03-20)
-------------------

1.11.0 (2014-02-21)
-------------------
* Use #!/usr/bin/env python for systems with multiple Python versions.
* Contributors: Benjamin Chretien

1.10.18 (2013-12-04)
--------------------

1.10.16 (2013-11-18)
--------------------

1.10.15 (2013-08-17)
--------------------

* joint_state_publisher: do not install script to global bin
  Also clean up no longer required setup.py
