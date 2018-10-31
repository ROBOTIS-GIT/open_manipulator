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

#ifndef OM_CHAIN_DRAWING_H_
#define OM_CHAIN_DRAWING_H_


#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <math.h>
#include <vector>

#include "robotis_manipulator/robotis_manipulator.h"

enum AXIS{
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ROLL,
    PITCH,
    YAW
};

using namespace ROBOTIS_MANIPULATOR;
using namespace Eigen;

namespace OM_CHAIN_DRAWING
{

class Line : public ROBOTIS_MANIPULATOR::DrawingTrajectory
{
private:
  WayPointType output_way_point_type_;

  std::vector<WayPoint> start_pose_;
  std::vector<WayPoint> goal_pose_;

  double acc_dec_time_;
  double move_time_;
  std::vector<double> vel_max_;

public:
  Line();
  virtual ~Line();

  void initLine(double move_time, double control_time, std::vector<WayPoint> start, std::vector<WayPoint> goal);
  std::vector<WayPoint> drawLine(double time_var);

  virtual void init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg);
  virtual std::vector<WayPoint> getJointWayPoint(double tick);
  virtual std::vector<WayPoint> getTaskWayPoint(double tick);
};

//-------------------- Circle --------------------//

class Circle : public ROBOTIS_MANIPULATOR::DrawingTrajectory
{
private:
  WayPointType output_way_point_type_;

  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
  VectorXd coefficient_;

  std::vector<WayPoint> start_pose_;
  std::vector<WayPoint> goal_pose_;

  double radius_;
  double start_angular_position_;
  double revolution_;

  double *get_arg_;

public:
  Circle();
  virtual ~Circle();

  void initCircle(double move_time, double control_time, std::vector<WayPoint> start, double radius, double revolution, double start_angular_position);
  std::vector<WayPoint> drawCircle(double time_var);
  MatrixXf getCoefficient();

  virtual void init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg);
  virtual std::vector<WayPoint> getJointWayPoint(double tick);
  virtual std::vector<WayPoint> getTaskWayPoint(double tick);
};


class Rhombus : public ROBOTIS_MANIPULATOR::DrawingTrajectory
{
private:
  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
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
  virtual void setgoalPose(Pose goal_pose);
  virtual void setAngularStartPosition(double start_angular_position);

  virtual Pose getPose(double tick);
};


class Heart : public ROBOTIS_MANIPULATOR::DrawingTrajectory
{
private:
  ROBOTIS_MANIPULATOR::MinimumJerk path_generator_;
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
  virtual void setgoalPose(Pose goal_pose);
  virtual void setAngularStartPosition(double start_angular_position);

  virtual Pose getPose(double tick);
};


} // namespace OM_CHAIN_DRAWING
#endif // OM_CHAIN_DRAWING_H_




