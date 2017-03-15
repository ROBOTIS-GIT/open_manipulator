/*
 *  Single Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2013, I Heart Engineering
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  William Morris <bill@iheartengineering.com>
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://www.iheartengineering.com
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AR_POSE_AR_SINGLE_H
#define AR_POSE_AR_SINGLE_H

#include <string.h>
#include <stdarg.h>

#include <artoolkit/AR/param.h>
#include <artoolkit/AR/ar.h>
#include <artoolkit/AR/video.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#if ROS_VERSION_MINIMUM(1, 9, 0)
  // new cv_bridge API in Groovy
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/image_encodings.h>
#else
  // Fuerte support for cv_bridge will be deprecated
  #if defined(__GNUC__)
    #warning "Support for the old cv_bridge API (Fuerte) is derecated and will be removed when Hydro is released."
  #endif
  #include <cv_bridge/CvBridge.h>
#endif


#include <ar_pose/ARMarker.h>

const std::string cameraImageTopic_ = "/camera/image_raw";
const std::string cameraInfoTopic_  = "/camera/camera_info";

const double AR_TO_ROS = 0.001;

namespace ar_pose
{
  class ARSinglePublisher
  {
  public:
    ARSinglePublisher (ros::NodeHandle & n);
    ~ARSinglePublisher (void);

  private:
    void arInit ();
    void getTransformationCallback (const sensor_msgs::ImageConstPtr &);
    void camInfoCallback (const sensor_msgs::CameraInfoConstPtr &);

    ros::NodeHandle n_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster broadcaster_;
    ros::Publisher arMarkerPub_;

    ar_pose::ARMarker ar_pose_marker_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber cam_sub_;
#if ! ROS_VERSION_MINIMUM(1, 9, 0)
    sensor_msgs::CvBridge bridge_;
#endif
    sensor_msgs::CameraInfo cam_info_;

    // **** parameters

    //std::string cameraFrame_;
    std::string markerFrame_;
    bool publishTf_;
    bool publishVisualMarkers_;
    bool useHistory_;
    int threshold_;
    double markerWidth_;        // Size of the AR Marker in mm

    ARParam cam_param_;         // Camera Calibration Parameters
    int patt_id_;               // AR Marker Pattern
    char pattern_filename_[FILENAME_MAX];
    bool reverse_transform_;    // Reverse direction of transform marker -> cam

    double marker_center_[2];   // Physical Center of the Marker
    double marker_trans_[3][4]; // Marker Transform

    // **** for visualisation in rviz
    ros::Publisher rvizMarkerPub_;
    visualization_msgs::Marker rvizMarker_;
    
    int contF;
    bool getCamInfo_;
    CvSize sz_;
#if ROS_VERSION_MINIMUM(1, 9, 0)
    cv_bridge::CvImagePtr capture_;
#else
    IplImage *capture_;
#endif

  };                            // end class ARSinglePublisher
}                               // end namespace ar_pose

#endif
