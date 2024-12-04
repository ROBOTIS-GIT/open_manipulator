/*******************************************************************************
* Copyright 2024 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim, Sungho Woo */

#ifndef OPEN_MANIPULATOR_X_GUI_QNODE_HPP_
#define OPEN_MANIPULATOR_X_GUI_QNODE_HPP_

#include <QThread>
#include <QStringListModel>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <std_srvs/srv/set_bool.hpp>

namespace open_manipulator_x_gui {

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPosition();
  bool setJointSpacePath(std::vector<double> joint_angle);
  bool setTaskSpacePath(
    std::vector<double> kinematics_pose,
    bool position_only,
    double position_tol,
    double orientation_tol);
  bool setToolControl(std::vector<double> joint_angle);
  bool isMotionComplete();
  void stopMotion();
  bool isStopRequested() ;
  void resetStopRequest();
  bool sendTorqueSrv(bool checked);
  std::string getCSVPath() const;

private:
  int init_argc;
  char** init_argv;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_thread_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group2_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematics_position_;

  void updateRobotState();
  std::atomic<bool> stop_requested_;
  std::string csv_file_path_;


Q_SIGNALS:
  void rosShutdown();
};

}  // namespace open_manipulator_x_gui

#endif /* OPEN_MANIPULATOR_X_GUI_QNODE_HPP_ */
