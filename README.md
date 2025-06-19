# 🦾 OpenMANIPULATOR

## Overview

This repository provides an integrated management package for **ROBOTIS** robotic arms, including:

- **OMY**
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

The **OMY** is a 6-DOF robotic arm designed for advanced robotic manipulation tasks. This ROS 2 package provides seamless integration, enhanced control, and versatile functionality for simulation and hardware applications.

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

   # Start container
   ./docker/container.sh start

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

2. **Install Intel RealSense ROS Wrapper**

   Please follow the official instructions for installing and using the RealSense ROS wrapper at:
   - https://github.com/IntelRealSense/realsense-ros

   This will ensure you have the latest and most compatible version for your system and camera.

3. **Clone the Repository**
   ```bash
   cd ~/ros2_ws/src
   git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
   git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
   git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
   git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git
   ```

4. **Install ROS 2 Dependencies**
   ```bash
   cd ~/ros2_ws
   rosdep update
   rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2 dynamixel_hardware_interface dynamixel_interfaces dynamixel_sdk open_manipulator" -y
   ```

5. **Build the Package**
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

6. **Source the Workspace**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

7. **(Optional) Add Convenience Alias**
   Add the following to your `~/.bashrc` for a convenient build alias:
   ```bash
   echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
   source ~/.bashrc
   ```

8. **Create and apply udev rules**
   ```bash
   ros2 run open_manipulator_bringup om_create_udev_rules
   ```

## **3. Launch Files Overview**

Below is a comprehensive list of available launch files, grouped by function. Use these to start the system in various modes, simulations, or with special features.

### **A. Hardware Launch (Real Robot)**

```bash
# Launch OMY 3M hardware
ros2 launch open_manipulator_bringup omy_3m.launch.py

# Launch OMY F3M hardware
ros2 launch open_manipulator_bringup omy_f3m.launch.py

# Launch OpenMANIPULATOR-X hardware
ros2 launch open_manipulator_bringup omx.launch.py

# Launch F3M follower for AI leader-follower mode
ros2 launch open_manipulator_bringup omy_f3m_follower_ai.launch.py

# Launch L100 as leader for AI leader-follower mode
ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py

# Launch the full AI teleoperation leader-follower stack (runs both leader and follower)
ros2 launch open_manipulator_bringup omy_ai.launch.py
```

### **B. Simulation (Gazebo)**

```bash
# Simulate OMY 3M in Gazebo
ros2 launch open_manipulator_bringup omy_3m_gazebo.launch.py

# Simulate OMY F3M in Gazebo
ros2 launch open_manipulator_bringup omy_f3m_gazebo.launch.py

# Simulate OpenMANIPULATOR-X in Gazebo
ros2 launch open_manipulator_bringup omx_gazebo.launch.py

# Simulate F3M follower in AI mode in Gazebo
ros2 launch open_manipulator_bringup omy_f3m_follower_ai_gazebo.launch.py
```

### **C. Specialized/Utility Launch**

```bash
# Packs the OMY 3M manipulator (can also be used for OMY F3M)
ros2 launch open_manipulator_bringup omy_3m_pack.launch.py

# Unpacks the OMY 3M manipulator (can also be used for OMY F3M)
ros2 launch open_manipulator_bringup omy_3m_unpack.launch.py

# Launch Intel RealSense camera nodes
ros2 launch open_manipulator_bringup camera_realsense.launch.py
```

### **D. GUI Launch**

```bash
# GUI for OMY 3M
ros2 launch open_manipulator_gui omy_3m_gui.launch.py

# GUI for OMY F3M
ros2 launch open_manipulator_gui omy_f3m_gui.launch.py

# GUI for OpenMANIPULATOR-X
ros2 launch open_manipulator_gui omx_gui.launch.py
```

### **E. MoveIt! Launch**

```bash
# MoveIt! for OMY 3M
ros2 launch open_manipulator_moveit_config omy_3m_moveit.launch.py

# MoveIt! for OMY F3M
ros2 launch open_manipulator_moveit_config omy_f3m_moveit.launch.py

# MoveIt! for OpenMANIPULATOR-X
ros2 launch open_manipulator_moveit_config omx_moveit.launch.py
```

### **F. Description Launch**

```bash
# Load robot description for OMY 3M
ros2 launch open_manipulator_description omy_3m.launch.py

# Load robot description for OMY F3M
ros2 launch open_manipulator_description omy_f3m.launch.py

# Load robot description for OMY L100
ros2 launch open_manipulator_description omy_l100.launch.py

# Load robot description for OpenMANIPULATOR-X
ros2 launch open_manipulator_description omx.launch.py
```

---

### **Mode Clarification**

- **Hardware Mode:** For real robot operation, use the hardware launch files from section A.
- **Simulation Mode:** For Gazebo simulation, use the launch files from section B.
- **AI Leader-Follower Mode:** Use `omy_ai.launch.py` to start both leader and follower, or launch `omy_f3m_follower_ai.launch.py` and `omy_l100_leader_ai.launch.py` separately for advanced setups.
- **Specialized Modes:** Use pack/unpack launch files for special poses, and camera launch for vision integration.

## (Legacy) ROBOTIS e-Manual for OpenMANIPULATOR-X

- [ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)

The **OpenMANIPULATOR-X operation method** is similar to **OMY**, and the e-Manual is currently being updated.
