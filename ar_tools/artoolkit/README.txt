Read me for ARToolKit-2.72.1.
=============================


Contents.
---------
About this archive.
The ARToolKit license model.
Building on Windows.
Building on Linux / SGI Irix.
Building on Mac OS X.
Changes in this release.
Known issues in this release.
Changes in earlier releases.


About this archive.
-------------------
This archive contains the ARToolKit libraries, utilities and examples, version 2.72.1.

ARToolKit version 2.72.1 is released under the GNU General Public License (GPL). Please read the file COPYING.txt.

The latest version of ARToolKit is available from http://sf.net/projects/artoolkit.

ARToolKit is designed to build on Linux, Windows, Macintosh OS X, and SGI Irix platforms.

This archive was assembled by:
    Philip Lamb
    HIT Lab NZ
    http://www.hitlabnz.org
    2007-02-07


The ARToolKit license model.
----------------------------

ARToolKit is made available under a dual-license model. As it has been since the first public release of version 1.0, ARToolKit is freely available for non-commercial use under the terms of the GNU General Public License. A proprietary version of ARToolKit developed in parallel for professional commercial use by ARToolworks, Inc. is made available under different license terms, to suit end-user need.

We believe this model offers the best of both worlds. Persons developing their software under an open source model are freely availed of the ARToolKit source code, and the ARToolKit open source project benefits in return from enhancements, bug reports, and external development ideas. Those persons developing under a proprietary source model, from whom the ARToolKit code base does not benefit through reciprocal openness, contribute financially instead. License fees fund research and development of today's and tomorrow's versions of the ARToolKit through research activities at partner institutions.

The GPL-licensed toolkit.:

ARToolKit versions 1.0 through 2.x are available under the GPL. Please read the full text of the GPL prior to downloading and/or using the ARToolKit source code. Your downloading and/or use of the code signifies acceptance of the terms and conditions of the license. If you are unable to comply with the license terms, please immediately destroy all copies of the code in your possession.

Please be aware that while the GPL allows you to freely use the source code, it also imposes certain restrictions on the way in which you can use the code. Your attention is drawn particularly to section 2b of the GPL: "You must cause any work that you distribute or publish, that in whole or in part contains or is derived from the Program or any part thereof, to be licensed as a whole at no charge to all third parties under the terms of this License.", i.e. your software incorporating or linking to ARToolKit must also be open-source software, licensed under the GPL. Use of the ARToolKit in breach of the terms of the GPL will be subject to legal action by the copyright holders.

Licenses for professional and commercial use.:

In response to demand from toolkit users, the holders of the copyright on much of the ARToolKit version 1.0 - 2.x code have elected to make this code and other substantially advanced ARToolKit and AR code available under proprietary licenses for professional and commercial use by persons for whom the GPL license is not ideal. These license arrangements are managed by ARToolworks, Inc., Seattle, WA, USA. A variety of license types are available at reasonable cost. Please contact ARToolworks, Inc. directly for information on products, license terms, and pricing.

ARToolworks also provide a variety of value-added services, including professional support, customization and other modifications, and end-user turnkey applications.


Building on Windows.
--------------------

Prerequisites:
 *  Microsoft Visual Studio .NET 2003 or Visual Studio 6, or a free development environment such as Cygwin.
 *  DSVideoLib-0.0.8b-win32. Download from http://sf.net/projects/artoolkit.
 *  GLUT. Download from http://www.xmission.com/~nate/glut/glut-3.7.6-bin.zip.
 *  (Optional, for VRML renderer only) OpenVRML-0.16.1-bin-win32. Download from http://sf.net/projects/artoolkit.
 
Build steps:
(1) Unpack the ARToolKit zip to a convenient location. This location will be referred to below as {ARToolKit}.
(2) Unpack the DSVideoLib zip into {ARToolKit}. Make sure that the directory is named "DSVL".
(3) Copy the files DSVL.dll and DSVLd.dll from {ARToolKit}\DSVL\bin into {ARToolKit}\bin.
(4) Install GLUT, following the instructions in the README.win file inside the GLUT zip. See http://www.hitlabnz.org/forum/showpost.php?p=332&postcount=12 for more detail on how to install GLUT on Windows.
(5) Run the script {ARToolKit}\Configure.win32.bat to create include\AR\config.h.
(6) Open the ARToolKit.sln file (VS.NET) or ARToolkit.dsw file (VS6).
(7) Build the toolkit.

The VRML renderering library and example (libARvrml & simpleVRML) are optional builds:
(8) Unpack the OpenVRML zip into {ARToolKit}.
(9) Copy js32.dll from {ARToolKit}\OpenVRML\bin into {ARToolKit}\bin.
(10) Enable the libARvrml and simpleVRML projects in the VS configuration manager and build.


Building on Linux / SGI Irix.
-----------------------------

Prerequisites:
 *  (Optional, for VRML renderer only) openvrml-0.16.1 and dependencies. Download from http://sf.net/projects/openvrml.
 
Unpack the ARToolKit to a convenient location. The root of this location will be referred to below as {ARToolKit}:
    tar zxvf ARToolKit-2.72.1.tgz
Configure and build. The Linux builds support video input using either Video4Linux, an IIDC-compliant or DV camera connected via IEEE-1394, or a Sony EyeToy camera connected via USB. Alternatively you can use GStreamer 0.10 (0.8 is not supported and also not recommended) as input method. This requires you to install the gstreamer development packages for your Linux distribution. You will be prompted as to which of the four Linux video drivers you wish to use at the Configure step.
    cd {ARToolKit}
    ./Configure
	make
Following a successful build, to run a binary such as simpleTest:
	cd {ARToolKit}/bin
	./simpleTest

The VRML renderering library and example (libARvrml & simpleVRML) are optional builds:
	cd {ARToolKit}/lib/SRC/ARvrml
	make
	cd {ARToolKit}/examples/simpleVRML
	make
	cd {ARToolKit}/bin
	./simpleVRML


Building on Mac OS X.
---------------------------

Prerequisites:
 *  Apple's Developer Tools. http://developer.apple.com
 *  Camera driver. Mac OS X 10.3 and later include an inbuilt IIDC and DV FireWire camera driver. The macam project provides USB camera drivers for OS X, http://webcam-osx.sf.net.
 *  (Optional, for VRML renderer only) Apple X11. Install X11 from the Mac OS X installer, from the "Optional installs" package,  or download an install disk image from http://www.apple.com/downloads/macosx/apple/x11formacosx.html.
 *  (Optional, for VRML renderer only) OpenVRML-0.16.16-bin-MacOSX.tar.gz. Download from http://sf.net/projects/artoolkit, or alternatively install via Fink. Download Fink (and optionally, FinkCommander) from http://fink.sourceforge.net/download/.

Building using the XCode IDE:
(1) Unpack the archive to a convenient location using StuffIt Expander, and open the ARToolKit.xproj.
(2) Builds include a script target "Configure" which enables accelerated and rectangular texturing by default. If you wish to change these defaults, manually run the ./Configure script from Terminal as for a command-line build.
(3) Mac OS X XCode builds now build the examples as bundled applications. The contents of the "Data" directory are copied into the application bundle at build time. The applications can thus be moved from their build locations. The utilities are still (mostly) built as command-line tools.

The VRML renderering library and example (libARvrml & simpleVRML) are optional builds:
(4) Unpack the OpenVRML .tar.gz into the ARToolKit folder, or alternatively, if using FinkCommander, do a binary install of mozilla-dev, followed by an install of openvrml4-dev and openvrml-gl5-dev.
(5) Select the ARToolKit extensions target, and build.

Alternately, ARToolKit can be built from the Terminal, using the Unix makefiles.
Drop the ARToolKit into a convenient location, e.g. your Desktop, then open a Terminal window and type:
	cd ~/Desktop
	tar zxvf ARToolKit-2.72.1.tgz
Configure and build
	cd ~/ARToolKit
	./Configure
	make
Following a successful build, to run a binary such as simpleTest, add these commands:
	cd bin
	./simpleTest

The VRML renderering library and example (libARvrml & simpleVRML) are optional builds:
	fink -b install mozilla-dev
	fink install openvrml6-dev openvrml-gl6-dev
	Then:
	cd ~/Desktop/ARToolKit/lib/SRC/ARvrml
	make
	cd ~/Desktop/ARToolKit/examples/simpleVRML
	make
	cd ~/Desktop/ARToolKit/bin
	./simpleVRML


Changes in version 2.72.1 (this release) (2007-02-07).
------------------------------------------------------
All platforms:
- Bug fix: Removed VRML backgrounds which were still showing when using new OpenVRML-0.16.3.
- Bug fix: The bud_B VRML model does not render correctly with OpenVRML-0.16.3 and has been temporarily removed.
- Enhancement: Debug mode ('d' key) and threshhold adjustment ('+' and '-' keys) are now enabled in all gsub_lite-based examples.

Windows:
- Bug fix: Fix for issue which made video stream appear white in examples based on the old gsub library.
- Enhancement: added reading of environment variable ARTOOLKIT_CONFIG for DSVL video capture.

Linux:
- Bug fix: VideoLinuxV4L should now compile correctly on 64-bit x86 systems.

Mac OS X:
- Bug fix: A long-standing performance problem in the Mac video library has been addressed, allowing correct and optimal buffering of the video stream.
- Enhancement: ARvrml now links to fink-supplied OpenVRML-0.16.3 by default.


Known issues in this release.
-----------------------------
- On Windows, it is not easy to work out what to put into the DSVideoLib xml config file. Particularly, DirectShow IDs may include a "&" character, which needs to be escaped in xml (i.e. changed to "&amp;" wherever it occurs).
- The Mac video library does not yet use the new QuickTime 7 video pipeline.
- Changing the pixel format requires recompilation of libAR. This problem is solved in ARToolKit Professional v4 (commercially-licensed).
- The Mac default pixel format has been changed back to ARGB for compatibility with the OSGART project release.


Changes in earlier releases.
----------------------------
Please see the file ChangeLog.txt.


--
EOF
