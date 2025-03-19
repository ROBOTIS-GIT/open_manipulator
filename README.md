# 🦾 OpenMANIPULATOR

## Overview

This repository provides an integrated management package for **Robotis** robotic arms, including:

- **OpenManipulator-Y**
- **OpenManipulator-X**
- **Leader-Follower Manipulator System**

With this integration, all Robotis manipulators are managed under a unified package to ensure better compatibility and functionality.

## Key Features

This package supports **Ubuntu 24.04 Jazzy** and **Gazebo Harmonic**, offering the following functionalities:

- **Graphical User Interface (GUI) Support**
- **Teleoperation (Teleop) Capabilities**
- **MoveIt! Integration for Motion Planning**
- **Leader-Follower Control Mechanism (with Gravity Compensation)**

------

# **Open Manipulator-Y User Guide**

## **1. Introduction**

The **Open Manipulator-Y** is a 6-DOF robotic arm designed for advanced robotic manipulation tasks. This ROS 2 package provides seamless integration, enhanced control, and versatile functionality for simulation and hardware applications.

------

## **2. Prerequisites**

### **Supported ROS Version**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)

This package is compatible only with **ROS 2 Jazzy**. Ensure that ROS 2 Jazzy is properly installed.

### **Required Packages**

Install the following dependencies:

```bash
sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-hardware-interface \
    libboost-all-dev \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-tf-transformations \
    ros-jazzy-gz* \
    ros-jazzy-pal-statistics
sudo apt-get install -y ros-jazzy-moveit-* --no-install-recommends
```

### **USB Port Permissions**

To enable communication with the hardware, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

### **Environment Configuration**

Set the robot model based on your system:

- **`om_y_follower`** – OpenManipulator-Y with leader-follower functionality.
- **`om_y`** – OpenManipulator-Y as a standalone follower model.
- **`om_x`** – OpenManipulator-X.

Add the configuration to `~/.bashrc`:

```bash
echo 'export ROBOT_MODEL=om_y_follower' >> ~/.bashrc
source ~/.bashrc
```

------

## **3. Installation**

### **1. Clone the Repository**

Navigate to your ROS 2 workspace and clone the repository:

```bash
cd ~/${WORKSPACE}/src

git clone -b feature-om-y-top-support https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
git clone -b jazzy https://github.com/ros-controls/gz_ros2_control
```

### **2. Build the Package**

Compile the package using `colcon`:

```bash
cd ~/${WORKSPACE}
colcon build --symlink-install
```

### **3. Source the Workspace**

```bash
source ~/${WORKSPACE}/install/setup.bash
```

Create and apply `udev` rules:

```bash
ros2 run open_manipulator_bringup y_create_udev_rules # for om_y
ros2 run open_manipulator_bringup x_create_udev_rules # for om_x
```

------

## **4. Execution Commands**

### **Step 1: Choose Your Operating Mode**

#### **1️⃣ Leader-Follower Mode**

For **leader-follower functionality**, use:

```bash
ros2 launch open_manipulator_bringup ai_teleoperation.launch.py 
```

Ensure proper connection and detection of leader and follower devices.

#### **2️⃣ Standalone Follower Mode**

For **standalone follower mode**, launch:

```bash
ros2 launch open_manipulator_bringup hardware_y.launch.py #for om_y
ros2 launch open_manipulator_bringup hardware_x.launch.py #for om_x
```

Confirm that hardware is properly connected before execution.

#### **3️⃣ Gazebo Simulation Mode**

For **Gazebo simulation mode**, launch:

```bash
bash
ros2 launch open_manipulator_bringup gazebo.launch.py #for om_x and om_y
```

Ensure that Gazebo Harmonic is properly installed and configured before running the simulation.

------

### **Step 2: Extend Functionality**

#### **1. Keyboard Teleoperation**

Control the manipulator (simulation or hardware) using your keyboard:

```bash
ros2 run open_manipulator_teleop keyboard_control_y.py # for om_y
ros2 run open_manipulator_teleop keyboard_control_x.py # for om_x
```

##### **Joint Control**

- `1` / `q` - Joint 1
- `2` / `w` - Joint 2
- `3` / `e` - Joint 3
- `4` / `r` - Joint 4
- `5` / `t` - Joint 5
- `6` / `y` - Joint 6

##### **Gripper Control**

- `o` - Open gripper
- `p` - Close gripper

#### **2. MoveIt! Launch**

Enable MoveIt functionality for advanced motion planning in RViz:

```bash
ros2 launch open_manipulator_moveit_config moveit_core.launch.py
```

Move interactive markers to position the robotic arm, then click **Plan** and **Execute**.

#### **3. GUI Control**

Launch MoveIt GUI:

```bash
ros2 launch open_manipulator_moveit_config move_group.launch.py
```

Launch the Open Manipulator GUI:

```bash
ros2 launch open_manipulator_gui open_manipulator_y_gui.launch.py # for om_y
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py # for om_x
```

------

### **Step 3: Explore GUI Features**

#### **Basic Controls**

- **Start Timer**: Activates the system.
- **Robot Status**: Displays current manipulator state.
- **Init Pose**: Moves the manipulator to a vertical position.
- **Home Pose**: Moves the manipulator to a compact, safe position.
- **Gripper Open/Close**: Opens or closes the gripper.

#### **Task Execution**

- **Joint Space Tab**: Adjust individual joint angles.

- **Task Space Tab**: Control the end-effector position.

- Task Constructor Tab

  :

  - **Read Task**: View saved poses.
  - **Save Pose**: Save current state.
  - **Rap**: Set task repetition (1–999).
  - **Play**: Execute saved tasks.
  - **Stop**: Halt operations.
  - **Reset Task**: Clear saved tasks.

------

## (Legacy) ROBOTIS e-Manual for OpenMANIPULATOR-X

- [ROBOTIS e-Manual](http://emanual.robotis.com/docs/en/platform/openmanipulator/)

The **OpenManipulator-X operation method** is similar to **OpenManipulator-Y**, and the e-Manual is currently being updated.
