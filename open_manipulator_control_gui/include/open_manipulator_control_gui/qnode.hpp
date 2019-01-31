/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef open_manipulator_control_gui_QNODE_HPP_
#define open_manipulator_control_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"

#define NUM_OF_JOINT_AND_TOOL 5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace open_manipulator_control_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
  bool getOpenManipulatorMovingState();
  bool getOpenManipulatorActuatorState();

  void setOption(std::string opt);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  bool setDrawingTrajectory(std::string name, std::vector<double> arg, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  bool setActuatorState(bool actuator_state);

Q_SIGNALS:
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  QStringListModel logging_model;

  ros::Publisher open_manipulator_option_pub_;

  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_joint_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;

  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient set_actuator_state_client_;
  ros::ServiceClient goal_drawing_trajectory_client_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  open_manipulator_msgs::KinematicsPose kinematics_pose_;

  bool open_manipulator_is_moving_;
  bool open_manipulator_actuator_enabled_;
};

}  // namespace open_manipulator_control_gui

#endif /* open_manipulator_control_gui_QNODE_HPP_ */
