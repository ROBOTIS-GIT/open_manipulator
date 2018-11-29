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

#ifndef open_manipulator_control_gui_MAIN_WINDOW_H
#define open_manipulator_control_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>
#include <eigen3/Eigen/Eigen>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace open_manipulator_control_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
  void writeLog(QString str);

public Q_SLOTS:
  void timerCallback();
  void on_btn_timer_start_clicked(void);
  void on_btn_init_pose_clicked(void);
  void on_btn_home_pose_clicked(void);
  void on_btn_gripper_open_clicked(void);
  void on_btn_gripper_close_clicked(void);
  void on_btn_read_joint_angle_clicked(void);
  void on_btn_send_joint_angle_clicked(void);
  void on_btn_read_kinematic_pose_clicked(void);
  void on_btn_send_kinematic_pose_clicked(void);
  void on_btn_set_gripper_clicked(void);
  void on_btn_actuator_enable_clicked(void);
  void on_btn_actuator_disable_clicked(void);
  void on_btn_get_manipulator_setting_clicked(void);
  void on_radio_drawing_line_clicked(void);
  void on_radio_drawing_circle_clicked(void);
  void on_radio_drawing_rhombus_clicked(void);
  void on_radio_drawing_heart_clicked(void);
  void on_btn_send_drawing_trajectory_clicked(void);

  void tabSelected();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  QTimer *timer;
};

}  // namespace open_manipulator_control_gui

#endif // open_manipulator_control_gui_MAIN_WINDOW_H
