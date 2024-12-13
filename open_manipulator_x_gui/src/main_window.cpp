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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/open_manipulator_x_gui/main_window.hpp"

namespace open_manipulator_x_gui
{

  using namespace Qt;

  MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc, argv)
  {
    ui.setupUi(this);
    tableWidget = new QTableWidget(this);
    tableWidget->setColumnCount(6);
    QStringList headers;
    headers << "Joint 1" << "Joint 2" << "Joint 3" << "Joint 4" << "Gripper" << "Status";
    tableWidget->setHorizontalHeaderLabels(headers);
    tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    ui.verticalLayout_19->addWidget(tableWidget);

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    // connect(ui.checkBox_2, &QCheckBox::toggled, this, &MainWindow::on_check_CheckBox_Toggled);

    ui.btn_init_pose->setEnabled(false);
    ui.btn_home_pose->setEnabled(false);
    ui.btn_gripper_open->setEnabled(false);
    ui.btn_gripper_close->setEnabled(false);
    ui.btn_read_joint_angle->setEnabled(false);
    ui.btn_send_joint_angle->setEnabled(false);
    ui.btn_read_kinematic_pose->setEnabled(false);
    ui.btn_send_kinematic_pose->setEnabled(false);
    ui.btn_set_gripper->setEnabled(false);
    ui.btn_save_pose->setEnabled(false);
    ui.btn_play->setEnabled(false);
    ui.btn_stop->setEnabled(false);
    ui.btn_reset_task->setEnabled(false);
    ui.btn_read_task->setEnabled(false);
    ui.tabWidget->setEnabled(false);

    ui.btn_timer_start->setEnabled(true);

    qnode.init();
  }

  MainWindow::~MainWindow() {}

  void MainWindow::timerCallback()
  {
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    ui.txt_j1->setText(QString::number(joint_angle.at(0), 'f', 3));
    ui.txt_j2->setText(QString::number(joint_angle.at(1), 'f', 3));
    ui.txt_j3->setText(QString::number(joint_angle.at(2), 'f', 3));
    ui.txt_j4->setText(QString::number(joint_angle.at(3), 'f', 3));
    ui.txt_grip->setText(QString::number(joint_angle.at(4), 'f', 3));

    std::vector<double> position = qnode.getPresentKinematicsPosition();

    if (position.size() != 7)
      return;

    ui.txt_x->setText(QString::number(position.at(0), 'f', 3));
    ui.txt_y->setText(QString::number(position.at(1), 'f', 3));
    ui.txt_z->setText(QString::number(position.at(2), 'f', 3));
    ui.txt_q_x->setText(QString::number(position.at(3), 'f', 3));
    ui.txt_q_y->setText(QString::number(position.at(4), 'f', 3));
    ui.txt_q_z->setText(QString::number(position.at(5), 'f', 3));
    ui.txt_q_w->setText(QString::number(position.at(6), 'f', 3));
  }
  void MainWindow::tabSelected()
  {
    if (ui.tabWidget->currentIndex() == 0)
      on_btn_read_joint_angle_clicked();
    if (ui.tabWidget->currentIndex() == 1)
      on_btn_read_kinematic_pose_clicked();
  }

  void MainWindow::writeLog(QString str)
  {
    ui.plainTextEdit_log->moveCursor(QTextCursor::End);
    ui.plainTextEdit_log->appendPlainText(str);
  }

  void MainWindow::on_btn_timer_start_clicked(void)
  {
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
    timer->start(100);
    writeLog("QTimer start : 100ms");
    ui.btn_timer_start->setEnabled(false);
    ui.btn_init_pose->setEnabled(true);
    ui.btn_home_pose->setEnabled(true);
    ui.btn_gripper_open->setEnabled(true);
    ui.btn_gripper_close->setEnabled(true);
    ui.btn_read_joint_angle->setEnabled(true);
    ui.btn_send_joint_angle->setEnabled(true);
    ui.btn_read_kinematic_pose->setEnabled(true);
    ui.btn_send_kinematic_pose->setEnabled(true);
    ui.btn_set_gripper->setEnabled(true);
    ui.btn_play->setEnabled(true);
    ui.btn_stop->setEnabled(true);
    ui.btn_reset_task->setEnabled(true);
    ui.btn_read_task->setEnabled(true);
    ui.tabWidget->setEnabled(true);

    csv_file_path_ = qnode.getCSVPath();

    qnode.start();
  }

  void MainWindow::on_btn_init_pose_clicked(void)
  {
    std::thread([this]()
    {
      std::vector<double> joint_angle = {0.0, 0.0, 0.0, 0.0};

      if (!qnode.setJointSpacePath(joint_angle))
      {
        QMetaObject::invokeMethod(this, [this]() {
          writeLog("[ERR!!] Failed to send service");
        }, Qt::QueuedConnection);
        return;
      }

      QMetaObject::invokeMethod(this, [this]() {
        writeLog("Send joint angle to initial pose");
      }, Qt::QueuedConnection);
    }).detach();
  }

  void MainWindow::on_btn_home_pose_clicked(void)
  {
    std::thread([this]()
    {
      std::vector<double> joint_angle = {0.0, -1.0, 0.7, 0.3};

      if (!qnode.setJointSpacePath(joint_angle))
      {
        QMetaObject::invokeMethod(this, [this]() {
          writeLog("[ERR!!] Failed to send service");
        }, Qt::QueuedConnection);
        return;
      }

      QMetaObject::invokeMethod(this, [this]() {
        writeLog("Send joint angle to home pose");
      }, Qt::QueuedConnection);
    }).detach();
  }

  void MainWindow::on_btn_gripper_open_clicked(void)
  {
    std::thread([this]()
    {
      std::vector<double> joint_angle = {0.019};

      if (!qnode.setToolControl(joint_angle))
      {
        QMetaObject::invokeMethod(this, [this]() {
          writeLog("[ERR!!] Failed to send service");
        }, Qt::QueuedConnection);
        return;
      }

      QMetaObject::invokeMethod(this, [this]() {
        writeLog("Send gripper open");
      }, Qt::QueuedConnection);
    }).detach();
  }

  void MainWindow::on_btn_gripper_close_clicked(void)
  {
    std::thread([this]()
    {
      std::vector<double> joint_angle = {-0.01};

      if (!qnode.setToolControl(joint_angle))
      {
          QMetaObject::invokeMethod(this, [this]() {
            writeLog("[WARN!!] Contacted with a object");
          }, Qt::QueuedConnection);
          return;
      }

      QMetaObject::invokeMethod(this, [this]() {
        writeLog("Send gripper close");
      }, Qt::QueuedConnection);
    }).detach();
  }

  void MainWindow::on_btn_read_joint_angle_clicked(void)
  {
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    ui.doubleSpinBox_j1->setValue(joint_angle.at(0));
    ui.doubleSpinBox_j2->setValue(joint_angle.at(1));
    ui.doubleSpinBox_j3->setValue(joint_angle.at(2));
    ui.doubleSpinBox_j4->setValue(joint_angle.at(3));

    ui.doubleSpinBox_gripper->setValue(joint_angle.at(4));

    writeLog("Read joint angle");
  }

  void MainWindow::on_btn_send_joint_angle_clicked(void)
  {
    std::vector<double> joint_angle;

    joint_angle.push_back(ui.doubleSpinBox_j1->value());
    joint_angle.push_back(ui.doubleSpinBox_j2->value());
    joint_angle.push_back(ui.doubleSpinBox_j3->value());
    joint_angle.push_back(ui.doubleSpinBox_j4->value());

    std::thread([this, joint_angle]()
    {
      bool success = qnode.setJointSpacePath(joint_angle);
      if (!success)
      {
        QMetaObject::invokeMethod(this, [this]() {
          writeLog("[ERR!!] Failed to send service");
        }, Qt::QueuedConnection);
      }
      else
      {
        QMetaObject::invokeMethod(this, [this]() {
            writeLog("Send joint angle");
        }, Qt::QueuedConnection);
      }
    }).detach();
  }

  void MainWindow::on_btn_read_kinematic_pose_clicked(void)
  {
    std::vector<double> position = qnode.getPresentKinematicsPosition();
    ui.doubleSpinBox_x->setValue(position.at(0));
    ui.doubleSpinBox_y->setValue(position.at(1));
    ui.doubleSpinBox_z->setValue(position.at(2));
    ui.doubleSpinBox_q_x->setValue(position.at(3));
    ui.doubleSpinBox_q_y->setValue(position.at(4));
    ui.doubleSpinBox_q_z->setValue(position.at(5));
    ui.doubleSpinBox_q_w->setValue(position.at(6));

    writeLog("Read task pose");
  }

  void MainWindow::on_btn_send_kinematic_pose_clicked(void)
  {
    std::vector<double> kinematics_pose;
    Eigen::Quaterniond temp_orientation;

    bool position_only = ui.checkBox->isChecked();
    double position_tol = ui.doubleSpinBox_position_tol->value();
    double orientation_tol = ui.doubleSpinBox_orientation_tol->value();

    kinematics_pose.push_back(ui.doubleSpinBox_x->value());
    kinematics_pose.push_back(ui.doubleSpinBox_y->value());
    kinematics_pose.push_back(ui.doubleSpinBox_z->value());
    kinematics_pose.push_back(ui.doubleSpinBox_q_x->value());
    kinematics_pose.push_back(ui.doubleSpinBox_q_y->value());
    kinematics_pose.push_back(ui.doubleSpinBox_q_z->value());
    kinematics_pose.push_back(ui.doubleSpinBox_q_w->value());

    std::thread([this, kinematics_pose, position_only, position_tol, orientation_tol]()
    {
      bool success = qnode.setTaskSpacePath(kinematics_pose, position_only, position_tol, orientation_tol);
      if (!success)
      {
        QMetaObject::invokeMethod(this, [this]() {
            writeLog("[ERR!!] Failed to send service");
        }, Qt::QueuedConnection);
      }
      else
      {
        QMetaObject::invokeMethod(this, [this]() {
            writeLog("Send task pose");
        }, Qt::QueuedConnection);
      }
    }).detach();
  }

  void MainWindow::on_btn_set_gripper_clicked(void)
  {
    std::vector<double> joint_angle;
    joint_angle.push_back(ui.doubleSpinBox_gripper->value());
    if (!qnode.setToolControl(joint_angle))
    {
      writeLog("[WARN!!] Contacted with a object");
      return;
    }
    writeLog("Send gripper value");
  }

  void MainWindow::on_btn_save_pose_clicked(void)
  {
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    std::ofstream file;
    file.open(csv_file_path_, std::ios::app);

    if (file.is_open())
    {
      file << joint_angle.at(0) << "," << joint_angle.at(1) << ","
        << joint_angle.at(2) << "," << joint_angle.at(3) << ","
        << joint_angle.at(4) << std::endl;
      file.close();

      writeLog("Pose saved to CSV.");

      int row = tableWidget->rowCount();
      tableWidget->insertRow(row);
      for (int i = 0; i < 5; ++i)
      {
        QTableWidgetItem *item = new QTableWidgetItem(QString::number(joint_angle.at(i), 'f', 3));
        tableWidget->setItem(row, i, item);
      }
      QTableWidgetItem *statusItem = new QTableWidgetItem("Saved");
      tableWidget->setItem(row, 5, statusItem);
    }
    else
    {
      writeLog("[ERR!!] Unable to open file.");
    }
  }

  void MainWindow::on_btn_read_task_clicked(void)
  {
    tableWidget->setRowCount(0);
    std::ifstream file(csv_file_path_);
    std::string line;

    if (file.is_open())
    {
      int row = 0;
      while (std::getline(file, line))
      {
        std::vector<std::string> values;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ','))
        {
          values.push_back(value);
        }

        if (values.size() == 5)
        {
          tableWidget->insertRow(row);
          for (int col = 0; col < 5; ++col)
          {
            double doubleValue = std::stod(values.at(col));
            QTableWidgetItem *item = new QTableWidgetItem(QString::number(doubleValue, 'f', 3));
            tableWidget->setItem(row, col, item);
          }
          QTableWidgetItem *statusItem = new QTableWidgetItem("Loaded");
          tableWidget->setItem(row, 5, statusItem);
          ++row;
        }
      }
      file.close();
      writeLog("Task data loaded and displayed.");
    }
    else
    {
      writeLog("[ERR!!] Unable to open file.");
    }
    ui.btn_save_pose->setEnabled(true);
  }

  void MainWindow::on_btn_play_clicked(void)
  {
    if (tableWidget->rowCount() == 0)
    {
      writeLog("[ERR!!] No tasks available in the table.");
      return;
    }

    qnode.resetStopRequest();

    int repeatCount = ui.txt_rap->value();

    std::thread([this, repeatCount]()
    {
      ui.btn_reset_task->setEnabled(false);
      for (int repeat = 0; repeat < repeatCount; ++repeat)
      {
        if (qnode.isStopRequested())
        {
          QMetaObject::invokeMethod(this, [this]()
          { writeLog("Play operation stopped by user."); }, Qt::QueuedConnection);
          break;
        }

        QMetaObject::invokeMethod(this, [this, repeat, repeatCount]()
        { writeLog(QString("Executing task repetition %1 of %2").arg(repeat + 1).arg(repeatCount)); }, Qt::QueuedConnection);

        int rowCount = tableWidget->rowCount();
        for (int row = 0; row < rowCount; ++row)
        {
          for (int col = 0; col < 6; ++col)
          {
            QMetaObject::invokeMethod(this, [this, row, col]()
            { tableWidget->item(row, col)->setBackground(Qt::white); }, Qt::QueuedConnection);
          }
          QMetaObject::invokeMethod(this, [this, row]()
          { tableWidget->item(row, 5)->setText("Pending"); }, Qt::QueuedConnection);
        }

        std::ifstream file(csv_file_path_);
        std::string line;
        std::vector<double> current_joint_angles = qnode.getPresentJointAngle();
        double previous_gripper_value = current_joint_angles.back();

        if (file.is_open())
        {
          int row = 0;
          while (std::getline(file, line))
          {
            if (qnode.isStopRequested())
            {
              QMetaObject::invokeMethod(this, [this]()
              { writeLog("Play operation stopped by user."); }, Qt::QueuedConnection);
              break;
            }

            std::vector<double> joint_angle;
            std::stringstream ss(line);
            std::string value;

            while (std::getline(ss, value, ','))
            {
              joint_angle.push_back(std::stod(value));
            }

            if (joint_angle.size() == 5)
            {
              double current_gripper_value = joint_angle.back();

              QMetaObject::invokeMethod(this, [this, row]()
              {
                for (int col = 0; col < 6; ++col)
                {
                  tableWidget->item(row, col)->setBackground(Qt::yellow);
                }
                tableWidget->item(row, 5)->setText("Executing..."); }, Qt::QueuedConnection);

              if (!qnode.setJointSpacePath(joint_angle))
              {
                QMetaObject::invokeMethod(this, [this]()
                { writeLog("[ERR!!] Failed to send joint space path service"); }, Qt::QueuedConnection);
                break;
              }

              if (std::abs(current_gripper_value - previous_gripper_value) > 0.01)
              {
                std::vector<double> gripper_only = {current_gripper_value};
                if (!qnode.setToolControl(gripper_only))
                {
                  QMetaObject::invokeMethod(this, [this]()
                  { writeLog("[ERR!!] Failed to send gripper control service"); }, Qt::QueuedConnection);
                  break;
                }

                QMetaObject::invokeMethod(this, [this]()
                { writeLog("Gripper adjusted."); }, Qt::QueuedConnection);
              }

              QMetaObject::invokeMethod(this, [this]()
              { writeLog("Playing saved pose."); }, Qt::QueuedConnection);

              QMetaObject::invokeMethod(this, [this, row]()
              {
                for (int col = 0; col < 6; ++col)
                {
                  tableWidget->item(row, col)->setBackground(Qt::green);
                }
                tableWidget->item(row, 5)->setText("Done"); }, Qt::QueuedConnection);

              previous_gripper_value = current_gripper_value;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            row++;
          }
          file.close();
        }
        else
        {
          QMetaObject::invokeMethod(this, [this]()
          { writeLog("[ERR!!] Unable to open file."); }, Qt::QueuedConnection);
        }
      }

      QMetaObject::invokeMethod(this, [this]()
      {
        writeLog("All repetitions completed. Resetting table.");
        int rowCount = tableWidget->rowCount();
        for (int row = 0; row < rowCount; ++row)
        {
          for (int col = 0; col < 6; ++col)
          {
            tableWidget->item(row, col)->setBackground(Qt::white);
          }
          tableWidget->item(row, 5)->setText("Done");
        }
        ui.btn_reset_task->setEnabled(true);
      }, Qt::QueuedConnection); })
    .detach();
  }


  void MainWindow::on_btn_stop_clicked(void)
  {
    qnode.stopMotion();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int rowCount = tableWidget->rowCount();
    for (int row = 0; row < rowCount; ++row)
    {
      for (int col = 0; col < 6; ++col)
      {
        tableWidget->item(row, col)->setBackground(Qt::white);
      }
      tableWidget->item(row, 5)->setText("Stopped");
    }
    writeLog("Robot motion stopped and state reset.");
  }

  void MainWindow::on_btn_reset_task_clicked(void)
  {
    std::ofstream file(csv_file_path_, std::ios::trunc);
    if (file.is_open())
    {
      file.close();
      writeLog("CSV file and log cleared.");
    }
    else
    {
      writeLog("[ERR!!] Unable to reset file.");
    }
    tableWidget->setRowCount(0);

    ui.plainTextEdit_log->clear();
    writeLog("Reset completed.");
  }

  // void MainWindow::on_check_CheckBox_Toggled(bool checked)
  // {
  //     writeLog(checked ? "Torque ON" : "Torque OFF");
  //     if (!qnode.sendTorqueSrv(checked))
  //       writeLog("[ERR!!] Failed to send torque service.");
  // }

} // namespace open_manipulator_x_gui
