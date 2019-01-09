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

#ifndef CUSTOM_TRAJECTORY_H_
#define CUSTOM_TRAJECTORY_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

//#define PI 3.141592

using namespace ROBOTIS_MANIPULATOR;
using namespace Eigen;

namespace CUSTOM_TRAJECTORY
{
enum AXIS{
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
};
//-------------------- Line --------------------//

class Line : public ROBOTIS_MANIPULATOR::CustomTaskTrajectory
{
private:
  TaskWayPoint start_pose_;
  TaskWayPoint goal_pose_;

  double acc_dec_time_;
  double move_time_;
  std::vector<double> vel_max_;

public:
  Line();
  virtual ~Line();

  void initLine(double move_time, TaskWayPoint start, TaskWayPoint delta);
  TaskWayPoint drawLine(double time_var);

  virtual void setOption(const void *arg);
  virtual void init(double move_time, TaskWayPoint start, const void *arg);
  virtual TaskWayPoint getTaskWayPoint(double tick);
};

//-------------------- Circle --------------------//

class Circle : public ROBOTIS_MANIPULATOR::CustomTaskTrajectory
{
private:
  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWayPoint start_pose_;
  TaskWayPoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;

public:
  Circle();
  virtual ~Circle();

  void initCircle(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
  TaskWayPoint drawCircle(double time_var);

  virtual void setOption(const void *arg);
  virtual void init(double move_time, TaskWayPoint start, const void *arg);
  virtual TaskWayPoint getTaskWayPoint(double tick);
};

//-------------------- Rhombus --------------------//

class Rhombus : public ROBOTIS_MANIPULATOR::CustomTaskTrajectory
{
private:
  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWayPoint start_pose_;
  TaskWayPoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;

public:
  Rhombus();
  virtual ~Rhombus();

  void initRhombus(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
  TaskWayPoint drawRhombus(double time_var);

  virtual void setOption(const void *arg);
  virtual void init(double move_time, TaskWayPoint start, const void *arg);
  virtual TaskWayPoint getTaskWayPoint(double tick);
};

//-------------------- Heart --------------------//

class Heart : public ROBOTIS_MANIPULATOR::CustomTaskTrajectory
{
private:
  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
  VectorXd coefficient_;

  TaskWayPoint start_pose_;
  TaskWayPoint goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;

public:
  Heart();
  virtual ~Heart();

  void initHeart(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
  TaskWayPoint drawHeart(double tick);

  virtual void setOption(const void *arg);
  virtual void init(double move_time, TaskWayPoint start, const void *arg);
  virtual TaskWayPoint getTaskWayPoint(double tick);
};


} // namespace CUSTOM_TRAJECTORY
#endif // CUSTOM_TRAJECTORY_H_




