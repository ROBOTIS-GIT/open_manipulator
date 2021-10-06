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

#include "../include/open_manipulator_x_libs/custom_trajectory.hpp"

using namespace custom_trajectory;
using namespace Eigen;

/*****************************************************************************
** Line
*****************************************************************************/
void Line::init_line(double move_time, TaskWaypoint start, TaskWaypoint delta)
{
  move_time_ = move_time;
  acc_dec_time_ = move_time_ * 0.2;
  vel_max_.resize(3);

  TaskWaypoint start_to_goal;

  start_pose_ = start;

  goal_pose_ .kinematic.orientation = start_pose_.kinematic.orientation;
  goal_pose_ .kinematic.position = start.kinematic.position + delta.kinematic.position;

  vel_max_.at(X_AXIS) = delta.kinematic.position(X_AXIS)/(move_time_ - acc_dec_time_);
  vel_max_.at(Y_AXIS) = delta.kinematic.position(Y_AXIS)/(move_time_ - acc_dec_time_);
  vel_max_.at(Z_AXIS) = delta.kinematic.position(Z_AXIS)/(move_time_ - acc_dec_time_);
}

TaskWaypoint Line::draw_line(double time_var)
{
  TaskWaypoint pose;

  if(acc_dec_time_ >= time_var)
  {
    pose.kinematic.position(X_AXIS) = 0.5*vel_max_.at(X_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.kinematic.position(X_AXIS);
    pose.kinematic.position(Y_AXIS) = 0.5*vel_max_.at(Y_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.kinematic.position(Y_AXIS);
    pose.kinematic.position(Z_AXIS) = 0.5*vel_max_.at(Z_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.kinematic.position(Z_AXIS);
  }
  else if(time_var > acc_dec_time_ && (move_time_ - acc_dec_time_) >= time_var )
  {
    pose.kinematic.position(X_AXIS) = vel_max_.at(X_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.kinematic.position(X_AXIS);
    pose.kinematic.position(Y_AXIS) = vel_max_.at(Y_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.kinematic.position(Y_AXIS);
    pose.kinematic.position(Z_AXIS) = vel_max_.at(Z_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.kinematic.position(Z_AXIS);
  }
  else if(time_var > (move_time_ - acc_dec_time_) && (time_var < move_time_))
  {
    pose.kinematic.position(X_AXIS) = goal_pose_.kinematic.position(X_AXIS) - vel_max_.at(X_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.kinematic.position(Y_AXIS) = goal_pose_.kinematic.position(Y_AXIS) - vel_max_.at(Y_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.kinematic.position(Z_AXIS) = goal_pose_.kinematic.position(Z_AXIS) - vel_max_.at(Z_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
  }
  else if(time_var <= move_time_)
  {
    pose.kinematic.position(X_AXIS) = goal_pose_.kinematic.position(X_AXIS);
    pose.kinematic.position(Y_AXIS) = goal_pose_.kinematic.position(Y_AXIS);
    pose.kinematic.position(Z_AXIS) = goal_pose_.kinematic.position(Z_AXIS);
  }

  pose.kinematic.orientation = start_pose_.kinematic.orientation;
  pose.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  return pose;
}

TaskWaypoint Line::getTaskWaypoint(double tick)
{
  return draw_line(tick);
}


void Line::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  TaskWaypoint *c_arg = (TaskWaypoint *)arg;
  init_line(move_time, start, c_arg[0]);
}

void Line::setOption(const void *arg) {}

/*****************************************************************************
** Circle
*****************************************************************************/
void Circle::init_circle(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position)
{
  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  Point drawingStart, drawingGoal;

  drawingStart.position = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.position = revolution_ * 2*M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time);
  coefficient_ = path_generator_.getCoefficient();
}

TaskWaypoint Circle::draw_circle(double tick)
{
  // get time variable
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  // set drawing trajectory
  TaskWaypoint pose;

  double diff_pose[2];

  diff_pose[0] = (cos(get_time_var)-1)*cos(start_angular_position_) - sin(get_time_var)*sin(start_angular_position_);
  diff_pose[1] = (cos(get_time_var)-1)*sin(start_angular_position_) + sin(get_time_var)*cos(start_angular_position_);

  pose.kinematic.position(X_AXIS) = start_pose_.kinematic.position(X_AXIS) + radius_ * diff_pose[0];
  pose.kinematic.position(Y_AXIS) = start_pose_.kinematic.position(Y_AXIS) + radius_ * diff_pose[1];
  pose.kinematic.position(Z_AXIS) = start_pose_.kinematic.position(Z_AXIS);

  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  pose.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  return pose;
}

TaskWaypoint Circle::getTaskWaypoint(double tick)
{
  return draw_circle(tick);
}

void Circle::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  init_circle(move_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}

void Circle::setOption(const void *arg){}

/*****************************************************************************
** Rhombus
*****************************************************************************/
void Rhombus::init_rhombus(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position)
{
  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  Point drawingStart, drawingGoal;

  drawingStart.position = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.position = revolution_ * 2*M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time);
  coefficient_ = path_generator_.getCoefficient();
}

TaskWaypoint Rhombus::draw_rhombus(double tick)
{
  // get time variable
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5); 

  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2];
  double traj[2];

  while(true)
  {
    if (get_time_var < PI*2) break;
    get_time_var = get_time_var - PI*2;
  }

  if (get_time_var >= 0 && get_time_var < PI/2){
    traj[0] = - get_time_var / (PI/2);
    traj[1] = - get_time_var / (PI/2);
  } else if (get_time_var >= PI/2 && get_time_var < PI){
    traj[0] = - get_time_var / (PI/2);
    traj[1] = get_time_var / (PI/2) - 2;
  } else if (get_time_var >= PI && get_time_var < PI*3/2){
    traj[0] = get_time_var / (PI/2) - 4;
    traj[1] = get_time_var / (PI/2) - 2;
  } else {
    traj[0] = get_time_var / (PI/2) - 4;
    traj[1] = - get_time_var / (PI/2) + 4;
  }
  
  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.kinematic.position(X_AXIS) = start_pose_.kinematic.position(X_AXIS) + radius_ * diff_pose[0];
  pose.kinematic.position(Y_AXIS) = start_pose_.kinematic.position(Y_AXIS) + radius_ * diff_pose[1];
  pose.kinematic.position(Z_AXIS) = start_pose_.kinematic.position(Z_AXIS);

  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  pose.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  return pose;
}

void Rhombus::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  init_rhombus(move_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}

TaskWaypoint Rhombus::getTaskWaypoint(double tick)
{
  return draw_rhombus(tick);
}
void Rhombus::setOption(const void *arg){}

/*****************************************************************************
** Heart
*****************************************************************************/
void Heart::init_heart(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position)
{
  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  Point drawingStart, drawingGoal;

  drawingStart.position = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.position = revolution_ * 2*M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time);
  coefficient_ = path_generator_.getCoefficient();
}

TaskWaypoint Heart::draw_heart(double tick)
{
  // get time variable
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  // set drawing trajectory
  TaskWaypoint pose;
  double diff_pose[2];
  double traj[2];

	double shift_offset = - 5.0;

  traj[0] = (shift_offset + (13*cos(get_time_var) - 5*cos(2*get_time_var) - 2*cos(3*get_time_var) - cos(4*get_time_var))) / 16;
  traj[1] = (16*sin(get_time_var)*sin(get_time_var)*sin(get_time_var)) / 16;

  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.kinematic.position(X_AXIS) = start_pose_.kinematic.position(X_AXIS) + radius_ * diff_pose[0];
  pose.kinematic.position(Y_AXIS) = start_pose_.kinematic.position(Y_AXIS) + radius_ * diff_pose[1];
  pose.kinematic.position(Z_AXIS) = start_pose_.kinematic.position(Z_AXIS);

  pose.kinematic.orientation = start_pose_.kinematic.orientation;

  pose.dynamic.linear.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.linear.acceleration = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.velocity = Eigen::Vector3d::Zero(3);
  pose.dynamic.angular.acceleration = Eigen::Vector3d::Zero(3);

  return pose;
}

void Heart::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  init_heart(move_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}

void Heart::setOption(const void *arg){}

TaskWaypoint Heart::getTaskWaypoint(double tick)
{
  return draw_heart(tick);
}
