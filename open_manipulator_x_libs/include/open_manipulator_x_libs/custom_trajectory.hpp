// Copyright 2019 ROBOTIS CO., LTD.
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
// Author: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na

#ifndef CUSTOM_TRAJECTORY_HPP
#define CUSTOM_TRAJECTORY_HPP

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

using namespace robotis_manipulator;
using namespace Eigen;

namespace custom_trajectory
{
enum AXIS{
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
};

/*****************************************************************************
** Line
*****************************************************************************/
class Line : public robotis_manipulator::CustomTaskTrajectory
{
 public:
	Line() {}
	virtual ~Line() {}

  void init_line(double move_time, TaskWaypoint start, TaskWaypoint delta);
  TaskWaypoint draw_line(double time_var);

  virtual void setOption(const void *arg);
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
  virtual TaskWaypoint getTaskWaypoint(double tick);

 private:
  TaskWaypoint start_pose_;
  TaskWaypoint goal_pose_;

  double acc_dec_time_;
  double move_time_;
  std::vector<double> vel_max_;
};

/*****************************************************************************
** Circle
*****************************************************************************/
class Circle : public robotis_manipulator::CustomTaskTrajectory
{
 public:
	Circle() {}
	virtual ~Circle() {}

  void init_circle(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
  TaskWaypoint draw_circle(double time_var);

  virtual void setOption(const void *arg);
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
  virtual TaskWaypoint getTaskWaypoint(double tick);

 private:
  robotis_manipulator::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWaypoint start_pose_;
  TaskWaypoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;
};

/*****************************************************************************
** Rhombus
*****************************************************************************/
class Rhombus : public robotis_manipulator::CustomTaskTrajectory
{
 public:
	Rhombus() {}
	virtual ~Rhombus() {}

  void init_rhombus(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
  TaskWaypoint draw_rhombus(double time_var);

  virtual void setOption(const void *arg);
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
  virtual TaskWaypoint getTaskWaypoint(double tick);

 private:
  robotis_manipulator::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWaypoint start_pose_;
  TaskWaypoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;
};

/*****************************************************************************
** Heart
*****************************************************************************/
class Heart : public robotis_manipulator::CustomTaskTrajectory
{
 public:
	Heart() {}
	virtual ~Heart() {}

  void init_heart(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
  TaskWaypoint draw_heart(double tick);

  virtual void setOption(const void *arg);
  virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
  virtual TaskWaypoint getTaskWaypoint(double tick);

 private:
  robotis_manipulator::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWaypoint start_pose_;
  TaskWaypoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;
};
} // namespace custom_trajectory
#endif // CUSTOM_TRAJECTORY_HPP




