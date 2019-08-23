#!/bin/bash
# Apache License 2.0
# Copyright (c) 2019, ROBOTIS CO., LTD.

echo ""
echo "[Note] OS version  >>> Ubuntu 18.04 (Bionic Beaver) or Linux Mint 19.x"
echo "[Note] Target ROS version >>> ROS Dashing Diademata"
echo "[Note] Robotis workspace   >>> $HOME/robotis_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target ROS version and name of robotis workspace]"
name_ros_version=${name_ros_version:="dashing"}
name_robotis_workspace=${name_robotis_workspace:="robotis_ws"}

echo "[Install ROS2]"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y ros-dashing-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool

echo "[Make the robotis workspace folder and test colcon build]"
source /opt/ros/$name_ros_version/setup.sh
mkdir -p $HOME/$name_robotis_workspace/src
cd $HOME/$name_robotis_workspace
colcon build --symlink-install

echo "[Set the ROS evironment]"
sh -c "echo \"alias gb='gedit ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_robotis_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_robotis_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd ~/$name_robotis_workspace && colcon build --symlink-install'\" >> ~/.bashrc"
sh -c "echo \"alias cr='cd ~/$name_robotis_workspace && rm -R build install log'\" >> ~/.bashrc"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_robotis_workspace/install/local_setup.bash\" >> ~/.bashrc"

echo "[Complete!!!]"
exit 0
