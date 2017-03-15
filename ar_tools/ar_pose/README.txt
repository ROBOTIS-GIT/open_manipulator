Read me for ar_pose package
===========================

Contents.
---------
About this packages.
Requirements.
Basic Usage.

------------------------------------------------------------
------------------------------------------------------------
About this pkg:
------------------------------------------------------------
ar_pose is a package for ROS that subscribe to usb_cam topics and using ARToolkit
published pose data (tf) from a camera and a(several) marker(s).
ar_pose is released under the GNU General Public License (GPL). Please read the file COPYING.txt.
 
Copyright (C) 2010, CCNY Robotics Lab
This pkg was assembled by:
	Ivan Dryanovski <ivan.dryanovski@gmail.com>
	William Morris <morris@ee.ccny.cuny.edu>
	Gautier Dumonteil <gautier.dumonteil@gmail.com>
	http://robotics.ccny.cuny.edu

------------------------------------------------------------
------------------------------------------------------------
Requirements:
------------------------------------------------------------
ar_pose requires ARToolKit libraries and include files to
be available in order to build. 
Make sure all packages dependances are check.

------------------------------------------------------------
------------------------------------------------------------
Basic Usage:
------------------------------------------------------------
Download ar_pose packages in your ROS packages directory
Then, in a shell:
	roscd ar_pose/
	rosmake
	
Run the examples:
	roscd ar_pose/
	cd demo/ar_single/
	./setup.sh
	roslaunch demo.launch

or/and
	cd demo/ar_multi/
	./setup.sh
	roslaunch demo.launch
