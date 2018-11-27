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

#include "../include/open_manipulator_libs/Drawing.h"

using namespace DRAWING;
using namespace Eigen;

//-------------------- Line --------------------//

Line::Line() {}
Line::~Line() {}

void Line::initLine(double move_time, double control_time, std::vector<WayPoint> start, std::vector<WayPoint> goal)
{
  output_way_point_type_ = TASK_WAY_POINT;
  move_time_ = move_time;
  acc_dec_time_ = move_time_ * 0.2;
  vel_max_.resize(3);

  std::vector<WayPoint> start_to_goal;
  start_to_goal.resize(6);

  start_pose_ = start;
  goal_pose_ = goal;

  for(int i = 0; i < 3; i ++)
  {
    start_to_goal.at(i).value = goal_pose_.at(i).value - start_pose_.at(i).value;
    vel_max_.at(i) = start_to_goal.at(i).value/(move_time_ - acc_dec_time_);
  }
}

std::vector<WayPoint> Line::drawLine(double time_var)
{
  std::vector<WayPoint> pose;
  pose.resize(6);

  if(acc_dec_time_ >= time_var) // acc time
  {
    pose.at(X_AXIS).value = 0.5*vel_max_.at(X_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.at(X_AXIS).value;
    pose.at(Y_AXIS).value = 0.5*vel_max_.at(Y_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.at(Y_AXIS).value;
    pose.at(Z_AXIS).value = 0.5*vel_max_.at(Z_AXIS)*pow(time_var, 2)/acc_dec_time_ + start_pose_.at(Z_AXIS).value;
  }
  else if(time_var > acc_dec_time_ && (move_time_ - acc_dec_time_) >= time_var )
  {
    pose.at(X_AXIS).value = vel_max_.at(X_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.at(X_AXIS).value;
    pose.at(Y_AXIS).value = vel_max_.at(Y_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.at(Y_AXIS).value;
    pose.at(Z_AXIS).value = vel_max_.at(Z_AXIS)*(time_var-(acc_dec_time_*0.5)) + start_pose_.at(Z_AXIS).value;
  }
  else if(time_var > (move_time_ - acc_dec_time_) && (time_var < move_time_))
  {
    pose.at(X_AXIS).value = goal_pose_.at(X_AXIS).value - vel_max_.at(X_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.at(Y_AXIS).value = goal_pose_.at(Y_AXIS).value - vel_max_.at(Y_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.at(Z_AXIS).value = goal_pose_.at(Z_AXIS).value - vel_max_.at(Z_AXIS)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
  }
  else if(time_var <= move_time_)
  {
    pose.at(X_AXIS).value = goal_pose_.at(X_AXIS).value;
    pose.at(Y_AXIS).value = goal_pose_.at(Y_AXIS).value;
    pose.at(Z_AXIS).value = goal_pose_.at(Z_AXIS).value;
  }

  pose.at(ROLL).value = start_pose_.at(ROLL).value;
  pose.at(PITCH).value = start_pose_.at(PITCH).value;
  pose.at(YAW).value = start_pose_.at(YAW).value;

  return pose;
}

std::vector<WayPoint> Line::getJointWayPoint(double tick)
{
  return {};
}
std::vector<WayPoint> Line::getTaskWayPoint(double tick)
{
  return drawLine(tick);
}


void Line::init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg)
{
  WayPoint *c_arg = (WayPoint *)arg;
  std::vector<WayPoint> goal;
  for(int i = 0; i < 6; i ++)
    goal.push_back(c_arg[i]);
  initLine(move_time, control_time, start, goal);
}
void Line::setOption(const void *arg)
{

}

//-------------------- Circle --------------------//

Circle::Circle() {}
Circle::~Circle() {}

void Circle::initCircle(double move_time, double control_time, std::vector<WayPoint> start, double radius, double revolution, double start_angular_position)
{
  output_way_point_type_ = TASK_WAY_POINT;

  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  WayPoint drawingStart, drawingGoal;

  drawingStart.value = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.value = revolution_ * 2*M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time, control_time);
  coefficient_ = path_generator_.getCoefficient();
}

std::vector<WayPoint> Circle::drawCircle(double tick)
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
  std::vector<WayPoint> pose;
  pose.resize(6);
  double diff_pose[2];

  diff_pose[0] = (cos(get_time_var)-1)*cos(start_angular_position_) - sin(get_time_var)*sin(start_angular_position_);
  diff_pose[1] = (cos(get_time_var)-1)*sin(start_angular_position_) + sin(get_time_var)*cos(start_angular_position_);

  pose.at(X_AXIS).value = start_pose_.at(X_AXIS).value + radius_ * diff_pose[0];
  pose.at(Y_AXIS).value = start_pose_.at(Y_AXIS).value + radius_ * diff_pose[1];
  pose.at(Z_AXIS).value = start_pose_.at(Z_AXIS).value;

  pose.at(ROLL).value = start_pose_.at(ROLL).value;
  pose.at(PITCH).value = start_pose_.at(PITCH).value;
  pose.at(YAW).value = start_pose_.at(YAW).value;

  return pose;
}

std::vector<WayPoint> Circle::getJointWayPoint(double tick)
{
  return {};
}
std::vector<WayPoint> Circle::getTaskWayPoint(double tick)
{
  return drawCircle(tick);
}

void Circle::init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  initCircle(move_time, control_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}
void Circle::setOption(const void *arg)
{

}

//-------------------- Rhombus --------------------//

Rhombus::Rhombus() {}
Rhombus::~Rhombus() {}  

void Rhombus::initRhombus(double move_time, double control_time, std::vector<WayPoint> start, double radius, double revolution, double start_angular_position)
{
  output_way_point_type_ = TASK_WAY_POINT;

  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  WayPoint drawingStart, drawingGoal;

  drawingStart.value = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.value = revolution_ * M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time, control_time);
  coefficient_ = path_generator_.getCoefficient();
}


std::vector<WayPoint> Rhombus::drawRhombus(double tick)
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
  std::vector<WayPoint> pose;
  pose.resize(6);
  double diff_pose[2];
  double traj[2];

  if (get_time_var >= 0 && get_time_var < PI/2){
    traj[0] = - get_time_var / (PI/2) * radius_;
    traj[1] = - get_time_var / (PI/2) * radius_;
  } else if (get_time_var >= PI/2 && get_time_var < PI){
    traj[0] = - get_time_var / (PI/2) * radius_;
    traj[1] = get_time_var / (PI/2) * radius_ - 2 * radius_;
  } else if (get_time_var >= PI && get_time_var < PI*3/2){
    traj[0] = get_time_var / (PI/2) * radius_ - 4 * radius_;
    traj[1] = get_time_var / (PI/2) * radius_ - 2 * radius_;
  } else {
    traj[0] = get_time_var / (PI/2) * radius_ - 4 * radius_;
    traj[1] = - get_time_var / (PI/2) * radius_ + 4 * radius_;
  }

  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.at(X_AXIS).value = start_pose_.at(X_AXIS).value + radius_ * diff_pose[0];
  pose.at(Y_AXIS).value = start_pose_.at(Y_AXIS).value + radius_ * diff_pose[1];
  pose.at(Z_AXIS).value = start_pose_.at(Z_AXIS).value;

  pose.at(ROLL).value = start_pose_.at(ROLL).value;
  pose.at(PITCH).value = start_pose_.at(PITCH).value;
  pose.at(YAW).value = start_pose_.at(YAW).value;

  return pose;
}


void Rhombus::init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  initRhombus(move_time, control_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}

std::vector<WayPoint> Rhombus::getJointWayPoint(double tick)
{
  return {};
}
std::vector<WayPoint> Rhombus::getTaskWayPoint(double tick)
{
  return drawRhombus(tick);
}
void Rhombus::setOption(const void *arg)
{

}

//-------------------- Heart --------------------//

Heart::Heart() {}
Heart::~Heart() {}

void Heart::initHeart(double move_time, double control_time, std::vector<WayPoint> start, double radius, double revolution, double start_angular_position)
{
  output_way_point_type_ = TASK_WAY_POINT;

  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  WayPoint drawingStart, drawingGoal;

  drawingStart.value = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;
  drawingStart.effort = 0.0;

  drawingGoal.value = revolution_ * M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;
  drawingGoal.effort = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time, control_time);
  coefficient_ = path_generator_.getCoefficient();
}

std::vector<WayPoint> Heart::drawHeart(double tick)
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
  std::vector<WayPoint> pose;
  pose.resize(6);
  double diff_pose[2];
  double traj[2];

  traj[0] =  - 1.0f/17.0f*radius_*7
    + (1.0f/17.0f*radius_*(13*cos(get_time_var) - 5*cos(2*get_time_var) - 2*cos(3*get_time_var) - cos(4*get_time_var)));
  traj[1] = 1.0f/17.0f*radius_*(16*sin(get_time_var)*sin(get_time_var)*sin(get_time_var));

  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.at(X_AXIS).value = start_pose_.at(X_AXIS).value + radius_ * diff_pose[0];
  pose.at(Y_AXIS).value = start_pose_.at(Y_AXIS).value + radius_ * diff_pose[1];
  pose.at(Z_AXIS).value = start_pose_.at(Z_AXIS).value;

  pose.at(ROLL).value = start_pose_.at(ROLL).value;
  pose.at(PITCH).value = start_pose_.at(PITCH).value;
  pose.at(YAW).value = start_pose_.at(YAW).value;

  return pose;
}

void Heart::init(double move_time, double control_time, std::vector<WayPoint> start, const void *arg)
{
  double *get_arg_ = (double *)arg;
  initHeart(move_time, control_time, start, get_arg_[0], get_arg_[1], get_arg_[2]);
}
void Heart::setOption(const void *arg)
{

}

std::vector<WayPoint> Heart::getJointWayPoint(double tick)
{
  return {};
}
std::vector<WayPoint> Heart::getTaskWayPoint(double tick)
{
  return drawHeart(tick);
}
