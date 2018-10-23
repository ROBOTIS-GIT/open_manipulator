/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef OMPATH_H_
#define OMPATH_H_


#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <math.h>
#include <vector>

#include "robotis_manipulator/OpenManipulator.h"

#define PI 3.141592
using namespace Eigen;

namespace OM_PATH
{

class Line : public OPEN_MANIPULATOR::Draw
{
private:
  Pose start_pose_;
  Pose end_pose_;
  double acc_dec_time_;
  double move_time_;
  Vector3f vel_max_;
  double *get_arg_;

public:
  Line();
  virtual ~Line();

  void init(double move_time, double control_time);
  Pose line(double time_var);

  virtual void initDraw(const void *arg);
  virtual void setStartPose(Pose start_pose);
  virtual void setEndPose(Pose end_pose);
  virtual Pose getPose(double tick);

  virtual void setRadius(double radius);
  virtual void setAngularStartPosition(double start_angular_position);
};

class Circle : public OPEN_MANIPULATOR::Draw
{
private:
  RM_TRAJECTORY::MinimumJerk path_generator_;
  MatrixXf coefficient_;

  uint8_t joint_num_;

  Vector3f start_position_;
  Pose start_pose_;
  double radius_;
  double start_angular_position_;

  double *get_arg_;

public:
  Circle();
  virtual ~Circle();

  void init(double move_time, double control_time);
  Pose circle(double time_var);

  MatrixXf getCoefficient();

  virtual void initDraw(const void *arg);
  virtual void setRadius(double radius);
  virtual void setStartPose(Pose start_pose);
  virtual void setEndPose(Pose end_pose);
  virtual void setAngularStartPosition(double start_angular_position);
  virtual Pose getPose(double tick);
};


class Rhombus : public OPEN_MANIPULATOR::Draw
{
private:
  RM_TRAJECTORY::MinimumJerk path_generator_;
  MatrixXf coefficient_;

  uint8_t joint_num_;

  Vector3f start_position_;
  Pose start_pose_;
  double radius_;
  double start_angular_position_;

  double *get_arg_;

public:
  Rhombus();
  virtual ~Rhombus();

  void init(double move_time, double control_time);
  Pose rhombus(double time_var);

  MatrixXf getCoefficient();

  virtual void initDraw(const void *arg);
  virtual void setRadius(double radius);
  virtual void setStartPose(Pose start_pose);
  virtual void setEndPose(Pose end_pose);
  virtual void setAngularStartPosition(double start_angular_position);

  virtual Pose getPose(double tick);
};


class Heart : public OPEN_MANIPULATOR::Draw
{
private:
  RM_TRAJECTORY::MinimumJerk path_generator_;
  MatrixXf coefficient_;

  uint8_t joint_num_;

  Vector3f start_position_;
  Pose start_pose_;
  double radius_;
  double start_angular_position_;

  double *get_arg_;

public:
  Heart();
  virtual ~Heart();

  void init(double move_time, double control_time);
  Pose heart(double time_var);

  MatrixXf getCoefficient();

  virtual void initDraw(const void *arg);
  virtual void setRadius(double radius);
  virtual void setStartPose(Pose start_pose);
  virtual void setEndPose(Pose end_pose);
  virtual void setAngularStartPosition(double start_angular_position);

  virtual Pose getPose(double tick);
};


} // namespace OM_PATH
#endif // OMPATH_H_




