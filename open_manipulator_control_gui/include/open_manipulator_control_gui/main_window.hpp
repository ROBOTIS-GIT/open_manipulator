/**
 * @file /include/open_manipulator_control_gui/main_window.hpp
 *
 * @brief Qt based gui for open_manipulator_control_gui.
 *
 * @date November 2010
 **/
#ifndef open_manipulator_control_gui_MAIN_WINDOW_H
#define open_manipulator_control_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
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
/**
 * @brief Qt central, all operations relating to the view part here.
 */
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
  void on_btn_gripper_off_clicked(void);
  void on_btn_gripper_on_clicked(void);
  void on_btn_read_joint_angle_clicked(void);
  void on_btn_send_joint_angle_clicked(void);
  void on_btn_read_kinematic_pose_clicked(void);
  void on_btn_send_kinematic_pose_clicked(void);
  void tabSelected();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  QTimer *timer;
};

}  // namespace open_manipulator_control_gui

#endif // open_manipulator_control_gui_MAIN_WINDOW_H
