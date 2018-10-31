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

#include "open_manipulator_libs/om_drawing.h"

using namespace OM_CHAIN_DRAWING;
using namespace Eigen;

//-------------------- Line --------------------//

Line::Line() {}
Line::~Line() {}

void Line::initLine(double move_time, double control_time, std::vector<WayPoint> start, std::vector<WayPoint> goal)
{
  output_way_point_type_ = TASK;
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

//-------------------- Circle --------------------//

Circle::Circle() {}
Circle::~Circle() {}

void Circle::initCircle(double move_time, double control_time, std::vector<WayPoint> start, double radius, double revolution, double start_angular_position)
{
  output_way_point_type_ = TASK;

  start_pose_ = start;

  radius_ = radius;
  revolution_ = revolution;
  start_angular_position_ = start_angular_position;

  WayPoint drawingStart, drawingGoal;

  drawingStart.value = 0.0;
  drawingStart.velocity = 0.0;
  drawingStart.acceleration = 0.0;

  drawingGoal.value = revolution_ * M_PI;
  drawingGoal.velocity = 0.0;
  drawingGoal.acceleration = 0.0;

  path_generator_.calcCoefficient(drawingStart, drawingGoal, move_time, control_time);
  coefficient_ = path_generator_.getCoefficient();
}

std::vector<WayPoint> Circle::drawCircle(double tick)
{
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

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
  get_arg_ = (double *)arg;

  init(get_arg_[0], get_arg_[1]);
}

//-------------------- Rhombus --------------------//

Rhombus::Rhombus() {}
Rhombus::~Rhombus() {}  

void Rhombus::init(double move_time, double control_time)
{
  Trajectory start;
  Trajectory goal;

  start.position = 0.0;    
  start.velocity = 0.0;
  start.acceleration = 0.0;

  goal.position = 2.03 * M_PI; 
  goal.velocity = 0.0;
  goal.acceleration = 0.0;

  path_generator_.calcCoefficient(start,
                                  goal,
                                  move_time,
                                  control_time);

  coefficient_ = path_generator_.getCoefficient();
}

Pose Rhombus::rhombus(double time_var)
{
  Pose pose;


  double traj[2];
  double diff_pose[2];

  if (time_var >= 0 && time_var < PI/2){
    traj[0] = - time_var / (PI/2) * radius_;
    traj[1] = - time_var / (PI/2) * radius_;
  } else if (time_var >= PI/2 && time_var < PI){ 
    traj[0] = - time_var / (PI/2) * radius_;
    traj[1] = time_var / (PI/2) * radius_ - 2 * radius_;
  } else if (time_var >= PI && time_var < PI*3/2){ 
    traj[0] = time_var / (PI/2) * radius_ - 4 * radius_;
    traj[1] = time_var / (PI/2) * radius_ - 2 * radius_;
  } else {
    traj[0] = time_var / (PI/2) * radius_ - 4 * radius_;
    traj[1] = - time_var / (PI/2) * radius_ + 4 * radius_;
  }

  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.position(0) = start_position_(0) + diff_pose[0];
  pose.position(1) = start_position_(1) + diff_pose[1];
  pose.position(2) = start_position_(2);

  pose.orientation = start_pose_.orientation;

  return pose;
}

void Rhombus::initDraw(const void *arg)
{
  get_arg_ = (double *)arg;

  init(get_arg_[0], get_arg_[1]);
}

void Rhombus::setAngularStartPosition(double start_angular_position)
{
  start_angular_position_ = start_angular_position;
}

void Rhombus::setRadius(double radius)
{
  radius_ = radius;
}
void Rhombus::setStartPose(Pose start_pose)
{
  start_pose_ = start_pose;
  start_position_ = start_pose.position;
}

void Rhombus::setgoalPose(Pose goal_pose){}
Pose Rhombus::getPose(double tick)
{
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  return rhombus(get_time_var);
}

//-------------------- Heart --------------------//

Heart::Heart() {}
Heart::~Heart() {}

void Heart::init(double move_time, double control_time)
{
  Trajectory start;
  Trajectory goal;

  start.position = 0.0;    
  start.velocity = 0.0;
  start.acceleration = 0.0;

  goal.position = 2 * M_PI; 
  goal.velocity = 0.0;
  goal.acceleration = 0.0;

  path_generator_.calcCoefficient(start,
                                  goal,
                                  move_time,
                                  control_time);

  coefficient_ = path_generator_.getCoefficient();
}

Pose Heart::heart(double time_var)
{
  Pose pose;

  double traj[2];
  double diff_pose[2];

  traj[0] =  - 1.0f/17.0f*radius_*7  
    + (1.0f/17.0f*radius_*(13*cos(time_var) - 5*cos(2*time_var) - 2*cos(3*time_var) - cos(4*time_var)));
  traj[1] = 1.0f/17.0f*radius_*(16*sin(time_var)*sin(time_var)*sin(time_var));

  diff_pose[0] = traj[0]*cos(start_angular_position_) - traj[1]*sin(start_angular_position_);
  diff_pose[1] = traj[0]*sin(start_angular_position_) + traj[1]*cos(start_angular_position_);

  pose.position(0) = start_position_(0) + diff_pose[0];
  pose.position(1) = start_position_(1) + diff_pose[1];
  pose.position(2) = start_position_(2);

  pose.orientation = start_pose_.orientation;

  return pose;
}

void Heart::initDraw(const void *arg)
{
  get_arg_ = (double *)arg;

  init(get_arg_[0], get_arg_[1]);
}

void Heart::setAngularStartPosition(double start_angular_position)
{
  start_angular_position_ = start_angular_position;
}

void Heart::setRadius(double radius)
{
  radius_ = radius;
}

void Heart::setStartPose(Pose start_pose)
{
  start_pose_ = start_pose;
  start_position_ = start_pose.position;
}
void Heart::setgoalPose(Pose goal_pose){}
Pose Heart::getPose(double tick)
{
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  return heart(get_time_var);
}
