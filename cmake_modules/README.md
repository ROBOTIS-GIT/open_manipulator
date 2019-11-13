cmake_modules
=============

A common repository for CMake Modules which are not distributed with CMake but are commonly used by ROS packages.

See the CONTRIBUTING.md file in this repository before submitting pull requests for new modules.

ROS Distros
-----------

This repository is intended to be used in the ROS 2 ament build system.

Provided Modules
----------------

1. [**NumPy**](http://www.numpy.org/) is the fundamental package for scientific computing with Python.
1. [**TBB**](https://www.threadingbuildingblocks.org/) lets you easily write parallel C++ programs that take full advantage of multicore performance.
1. [**TinyXML**](http://www.grinninglizard.com/tinyxml/) is a simple, small, C++ XML parser.
1. [**TinyXML2**](http://www.grinninglizard.com/tinyxml2/) is a simple, small, C++ XML parser, continuation of TinyXML.
1. [**Xenomai**](http://www.xenomai.org/) is a real-time development framework cooperating with the Linux kernel.
1. [**GSL**](http://www.gnu.org/software/gsl/) is a numerical library for C and C++ programmers.
1. [**Gflags**](https://gflags.github.io/gflags/) is a C++ library that implements commandline flags processing with the ability to define flags in the source file in which they are used.

Usage
-----

To use the CMake modules provided by this ament package, you must `<build_depend>` on it in your `package.xml`, like so:

```xml
<?xml version="1.0"?>
<package>
  <!-- ... -->
  <build_depend>cmake_modules</build_depend>
</package>
```

Then you must `find_package` the cmake_modules package directly:

```cmake
find_package(cmake_modules REQUIRED)
```

After the above `find_package` invocations, the modules provided by `cmake_modules` will be available in your `CMAKE_MODULE_PATH` to be found. For example you can find `TinyXML` by using the following:

```cmake
find_package(TinyXML REQUIRED)
```

### Lookup sheet

##### NumPY

```cmake
find_package(NUMPY REQUIRED)
```

##### TBB

```cmake
find_package(TBB REQUIRED)
```

##### TinyXML

```cmake
find_package(TinyXML REQUIRED)
```

##### Xenomai

```cmake
find_package(Xenomai REQUIRED)
```

### FindGSL

```cmake
find_package(GSL REQUIRED)
```

##### Gflags

```cmake
find_package(Gflags REQUIRED)
```
