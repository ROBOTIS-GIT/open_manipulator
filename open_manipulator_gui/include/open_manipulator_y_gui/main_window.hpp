// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Ryan Shim, Sungho Woo, Wonho Yoon, Woojin Wie

#ifndef OPEN_MANIPULATOR_Y_GUI__MAIN_WINDOW_HPP_
#define OPEN_MANIPULATOR_Y_GUI__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QTimer>
#include <QTableWidget>
#include <QHeaderView>

#include <eigen3/Eigen/Eigen>

#include <cstdio>
#include <cstring>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "open_manipulator_y_gui/qnode.hpp"
#include "open_manipulator_y_gui/ui_y_main_window.h"


namespace open_manipulator_y_gui
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char ** argv, QWidget * parent = 0);
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
  void on_btn_save_pose_clicked(void);
  void on_btn_play_clicked(void);
  void on_btn_reset_task_clicked(void);
  void on_btn_stop_clicked(void);
  void on_btn_read_task_clicked(void);
  void tabSelected();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  QTimer * timer;
  QTableWidget * tableWidget;
  std::string csv_file_path_;
};

}  // namespace open_manipulator_y_gui

#endif  // OPEN_MANIPULATOR_Y_GUI__MAIN_WINDOW_HPP_
