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

#include "open_manipulator_libs/om_link_drawing.h"

using namespace OM_CHAIN_DRAWING;
using namespace Eigen;


//-------------------- Line --------------------//

Line::Line() {}

Line::~Line() {}

void Line::init(double move_time, double control_time)
{
  move_time_ = move_time;
  acc_dec_time_ = move_time_ * 0.2;

  Vector3f start_to_end;
  start_to_end(0) = end_pose_.position(0) - start_pose_.position(0);
  start_to_end(1) = end_pose_.position(1) - start_pose_.position(1);
  start_to_end(2) = end_pose_.position(2) - start_pose_.position(2);

  vel_max_(0) = start_to_end(0)/(move_time_ - acc_dec_time_);
  vel_max_(1) = start_to_end(1)/(move_time_ - acc_dec_time_);
  vel_max_(2) = start_to_end(2)/(move_time_ - acc_dec_time_);
}

Pose Line::line(double time_var)
{
  Pose pose;

  if(acc_dec_time_ >= time_var) // acc time
  {
    pose.position(0) = 0.5*vel_max_(0)*pow(time_var, 2)/acc_dec_time_ + start_pose_.position(0);
    pose.position(1) = 0.5*vel_max_(1)*pow(time_var, 2)/acc_dec_time_ + start_pose_.position(1);
    pose.position(2) = 0.5*vel_max_(2)*pow(time_var, 2)/acc_dec_time_ + start_pose_.position(2);
  }
  else if(time_var > acc_dec_time_ && (move_time_ - acc_dec_time_) >= time_var )
  {
    pose.position(0) = vel_max_(0)*(time_var-(acc_dec_time_*0.5)) + start_pose_.position(0);
    pose.position(1) = vel_max_(1)*(time_var-(acc_dec_time_*0.5)) + start_pose_.position(1);
    pose.position(2) = vel_max_(2)*(time_var-(acc_dec_time_*0.5)) + start_pose_.position(2);
  }
  else if(time_var > (move_time_ - acc_dec_time_) && (time_var < move_time_))
  {
    pose.position(0) = end_pose_.position(0) - vel_max_(0)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.position(1) = end_pose_.position(1) - vel_max_(1)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
    pose.position(2) = end_pose_.position(2) - vel_max_(2)*0.5/acc_dec_time_*(pow((move_time_-time_var),2));
  }
  else if(time_var <= move_time_)
  {
    pose.position(0) = end_pose_.position(0);
    pose.position(1) = end_pose_.position(1);
    pose.position(2) = end_pose_.position(2);
  }

  pose.orientation = start_pose_.orientation;

  return pose;
}

void Line::setStartPose(Pose start_pose)
{
  start_pose_ = start_pose;
}

void Line::setEndPose(Pose end_pose)
{
  end_pose_ = end_pose;
}
Pose Line::getPose(double tick)
{
  return line(tick);
}

void Line::initDraw(const void *arg)
{
  get_arg_ = (double *)arg;

  init(get_arg_[0], get_arg_[1]);
}
void Line::setRadius(double radius){}
void Line::setAngularStartPosition(double start_angular_position){}

//-------------------- Circle --------------------//

Circle::Circle() {}

Circle::~Circle() {}

void Circle::init(double move_time, double control_time)
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

void Circle::setAngularStartPosition(double start_angular_position)
{
  start_angular_position_ = start_angular_position;
}

void Circle::setRadius(double radius)
{
  radius_ = radius;
}
void Circle::setStartPose(Pose start_pose)
{
  start_pose_ = start_pose;
  start_position_ = start_pose.position;
}

void Circle::setEndPose(Pose end_pose){}

Pose Circle::circle(double time_var)
{
  Pose pose;
  double diff_pose[2];

  diff_pose[0] = (cos(time_var)-1)*cos(start_angular_position_) - sin(time_var)*sin(start_angular_position_);
  diff_pose[1] = (cos(time_var)-1)*sin(start_angular_position_) + sin(time_var)*cos(start_angular_position_);

  pose.position(0) = start_position_(0) + radius_ * diff_pose[0];
  pose.position(1) = start_position_(1) + radius_ * diff_pose[1];
  pose.position(2) = start_position_(2);

  pose.orientation = start_pose_.orientation;

  return pose;
}

Pose Circle::getPose(double tick)
{
  double get_time_var = 0.0;

  get_time_var = coefficient_(0) +
                 coefficient_(1) * pow(tick, 1) +
                 coefficient_(2) * pow(tick, 2) +
                 coefficient_(3) * pow(tick, 3) +
                 coefficient_(4) * pow(tick, 4) +
                 coefficient_(5) * pow(tick, 5);

  return circle(get_time_var);
}

void Circle::initDraw(const void *arg)
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

void Rhombus::setEndPose(Pose end_pose){}
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
void Heart::setEndPose(Pose end_pose){}
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
