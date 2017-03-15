/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * ar_manipulator_demo.cpp
 *
 *  Created on: Jan 19, 2017
 *      Author: sch, Darby Lim
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sstream>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <ar_pose/ARMarker.h>
#include <ar_pose/ARMarkers.h>

ros::Publisher ar_tf_pub;
tf::TransformListener *tf_listener;

void arTransformMsgCallback( const ar_pose::ARMarker::ConstPtr& msg )
{
  geometry_msgs::Pose pose_msg;

  tf::StampedTransform transform;

  try
  {
    tf_listener->lookupTransform( "/world", "/ar_marker", ros::Time(0), transform );

    pose_msg.position.x = transform.getOrigin().x();
    pose_msg.position.y = transform.getOrigin().y();
    pose_msg.position.z = transform.getOrigin().z();

    pose_msg.orientation.x = transform.getRotation().x();
    pose_msg.orientation.y = transform.getRotation().y();
    pose_msg.orientation.z = transform.getRotation().z();
    pose_msg.orientation.w = transform.getRotation().w();

    ar_tf_pub.publish(pose_msg);
  }
  catch (tf::TransformException ex)
  {
    return ;
  }
}

int main( int argc , char **argv )
{
  ros::init( argc , argv , "ar_manipulator_demo" );
  ros::NodeHandle nh("~");

  ar_tf_pub = nh.advertise<geometry_msgs::Pose>("/robotis/open_manipulator/ar_transform", 10);

  ros::Subscriber ar_pose_marker_sub = nh.subscribe("/ar_pose_marker", 10, arTransformMsgCallback);

  tf_listener = new tf::TransformListener();

  ros::spin();

  return 0;
}
