# 🦾 OpenMANIPULATOR

## Overview

This repository provides an integrated management package for **ROBOTIS** robotic arms, including:

- **OpenMANIPULATOR-Y**
- **OpenMANIPULATOR-X**
- **Leader-Follower Manipulator System**

With this integration, all Robotis manipulators are managed under a unified package to ensure better compatibility and functionality.

## Key Features

This package supports **ROS 2 Jazzy** and **Gazebo Harmonic** on Ubuntu 24.04, offering the following functionalities:

- **Integration of MoveIt 2 for Enhanced Motion Planning**
- **Graphical User Interface (GUI) Implementation and Support**
- **Teleoperation (Teleop) Capabilities**
- **Leader-Follower Control Mechanism with Gravity Compensation for Imitation Learning**

# **OpenMANIPULATOR User Guide**

## **1. Introduction**

The **OpenMANIPULATOR-Y** is a 6-DOF robotic arm designed for advanced robotic manipulation tasks. This ROS 2 package provides seamless integration, enhanced control, and versatile functionality for simulation and hardware applications.

## **2. Installation Methods**

You can choose between two installation methods:

### **Option 1: Using Docker (Recommended)**

This method provides an isolated environment with all dependencies pre-installed.

1. **Install Docker and Docker Compose**
   Follow the official Docker installation guide: [Install Docker Engine](https://docs.docker.com/engine/install/)

2. **Clone the Repository**
   ```bash
   git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
   cd open_manipulator
   ```

3. **Container Management**
   The repository includes a container management script with the following commands:

   ```bash
   # Show help
   ./docker/container.sh help

   # Start container with Gazebo support
   ./docker/container.sh start with_gz

   # Start container without Gazebo support
   ./docker/container.sh start without_gz

   # Enter the running container
   ./docker/container.sh enter

   # Stop and remove the container
   ./docker/container.sh stop
   ```

   [***Note***] <u>When stopping the container, you'll be asked for confirmation as this will remove all unsaved data in the container.</u>

4. **Data Persistence**
   The container maps the following directories for data persistence:
   - `./docker/workspace:/workspace` - The workspace directory inside the docker folder is mapped to `/workspace` inside the container
   
   [***Important***] <u>Data Persistence Rules:
   - Data in `/workspace` inside the container is saved to `docker/workspace` on your host
   - Container restart (using `docker restart`) maintains all data
   - Container removal (using `container.sh stop`) will remove all data except what's in the mapped `/workspace` directory
   - Always save your work in the `/workspace` directory to ensure it persists after container removal</u>

### **Option 2: Host Installation**

Follow these steps if you prefer to install directly on your host system:

1. **Prerequisites**

   - **Supported ROS Version**
     ![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
     This package is compatible only with **ROS 2 Jazzy**. Ensure that ROS 2 Jazzy is properly installed.

   - **USB Port Permissions**
     To enable communication with the hardware, add your user to the `dialout` group:
     ```bash
     sudo usermod -aG dialout $USER
     ```
     **A login and logout are required.**

   - **Environment Configuration**
     Set the robot model based on your system:
     - **`om_y_follower`** – OpenMANIPULATOR-Y with leader-follower functionality.
     - **`om_y`** – OpenMANIPULATOR-Y as a standalone model.
     - **`om_x`** – OpenMANIPULATOR-X.

     [***Caution***] <u>Make sure to configure it properly before using the desired mode.</u>

     ex) Add the configuration to `~/.bashrc`:
     ```bash
     echo 'export ROBOT_MODEL=om_y' >> ~/.bashrc
     source ~/.bashrc
     ```

2. **Install Required Packages**
   ```bash
   sudo apt-get update && sudo apt-get install -y \
       libboost-all-dev \
       ros-jazzy-hardware-interface \
       ros-jazzy-controller-manager \
       ros-jazzy-ros2-controllers \
       ros-jazzy-tf-transformations \
       ros-jazzy-gz* \
       ros-jazzy-pal-statistics
   sudo apt-get install -y ros-jazzy-moveit-* --no-install-recommends
   ```

3. **Clone the Repository**
   ```bash
   cd ~/${WORKSPACE}/src
   git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
   git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
   git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
   ```

4. **Build the Package**
   ```bash
   cd ~/${WORKSPACE}
   colcon build --symlink-install
   ```

5. **Source the Workspace**
   ```bash
   source ~/${WORKSPACE}/install/setup.bash
   ```

6. **Create and apply udev rules**
   ```bash
   ros2 run open_manipulator_bringup om_create_udev_rules
   ```

## **3. Execution Commands**

### **Step 1: Choose Your Operating Mode**

#### **1️⃣ Leader-Follower Mode**

For **leader-follower functionality**, use:

```bash
ros2 launch open_manipulator_bringup ai_teleoperation.launch.py
```

Ensure proper connection and detection of leader and follower devices.

#### **2️⃣ Standalone Mode**

For **standalone mode**, launch:

```bash
ros2 launch open_manipulator_bringup hardware_y.launch.py #for om_y
ros2 launch open_manipulator_bringup hardware_x.launch.py #for om_x
```

Confirm that hardware is properly connected before execution.

#### **3️⃣ Gazebo Simulation Mode**

For **Gazebo simulation mode**, launch:

```bash
ros2 launch open_manipulator_bringup gazebo.launch.py #for om_x and om_y
```

Ensure that Gazebo Harmonic is properly installed and configured before running the simulation.


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

Launch the OpenMANIPULATOR GUI:

```bash
ros2 launch open_manipulator_gui open_manipulator_y_gui.launch.py # for om_y
ros2 launch open_manipulator_gui open_manipulator_x_gui.launch.py # for om_x
```


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

  - **Read Task**: View saved poses.
  - **Save Pose**: Save current state.
  - **Rap**: Set task repetition (1–999).
  - **Play**: Execute saved tasks.
  - **Stop**: Halt operations.
  - **Reset Task**: Clear saved tasks.


## (Legacy) ROBOTIS e-Manual for OpenMANIPULATOR-X

- [ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)

The **OpenMANIPULATOR-X operation method** is similar to **OpenMANIPULATOR-Y**, and the e-Manual is currently being updated.
