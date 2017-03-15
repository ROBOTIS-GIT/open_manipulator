^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2015-03-23)
------------------
* Fixed changelog links to point to main repo
* Contributors: Shaun Edwards

0.4.0 (2015-03-21)
------------------
* Moved common socket contstructor code to simple_socket base class
* Updated simple message header to reflect vendor ranges specified in REP-I0004
* Correctly initialized connected state for udp connections
* Fixed issue `#48 <https://github.com/ros-industrial/industrial_core/issues/48>`_, logSocketError is now passed errno
* Merge pull request `#70 <https://github.com/ros-industrial/industrial_core/issues/70>`_ from gt-ros-pkg/hydro-devel
  Fixing receiveBytes for UDP
* Macro'ed out GETHOSTBYNAME, and fixed if-statement braces to be on a new line for consistency
* Added support for gethostbyname, for passing host names in addition to IP addresses.
* Making setConnected protected again, adding setDisconnected to public methods so that that method can be used to flag the connection as disconnected.
* Putting back in timeout for receiveBytes
* More formal fix for UDP communication.
  This should now make UDP sockets act almost exactly like the
  TCP sockets.
* Fixing receiveBytes for UDP
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: Fred Proctor, Kelsey, Shaun Edwards, gavanderhoorn

0.3.4 (2014-01-21)
------------------
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: gavanderhoorn

0.3.3 (2014-01-13)
------------------
* Fixed build issue due simple message library linking
* Contributors: gavanderhoorn

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* Added polling check to socket read and muiltiple read calls in order to receive all desired bytes
* Removed library export from catkin macro.  Packages that depend on these must declare library dependencies explicitly (by name)
* Add error message to socket errors (instead of just errno).
* Converted to catkin
* Contributors: Christina Gomez, JeremyZoss, ROS, Shaun Edwards, gavanderhoorn, jrgnicho, kphawkins, ros-industrial
