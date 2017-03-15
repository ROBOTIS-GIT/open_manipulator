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

#include "ar_pose/ar_single.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_single");
  ros::NodeHandle n;
  ar_pose::ARSinglePublisher arSingle(n);
  ros::spin();
  return 0;
}

namespace ar_pose
{
  ARSinglePublisher::ARSinglePublisher (ros::NodeHandle & n):n_ (n), it_ (n_)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
	std::string default_path = "data/patt.hiro";
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    ROS_INFO("Starting ArSinglePublisher");

    // **** get parameters

    if (!n_param.getParam("publish_tf", publishTf_))
      publishTf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    if (!n_param.getParam("publish_visual_markers", publishVisualMarkers_))
      publishVisualMarkers_ = true;
    ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

    if (!n_param.getParam("threshold", threshold_))
      threshold_ = 100;
    ROS_INFO ("\tThreshold: %d", threshold_);

    if (!n_param.getParam("marker_width", markerWidth_))
      markerWidth_ = 80.0;
    ROS_INFO ("\tMarker Width: %.1f", markerWidth_);

    if (!n_param.getParam("reverse_transform", reverse_transform_))
      reverse_transform_ = false;
    ROS_INFO("\tReverse Transform: %d", reverse_transform_);

    if (!n_param.getParam("marker_frame", markerFrame_))
      markerFrame_ = "ar_marker";
    ROS_INFO ("\tMarker frame: %s", markerFrame_.c_str());

    // If mode=0, we use arGetTransMat instead of arGetTransMatCont
    // The arGetTransMatCont function uses information from the previous image
    // frame to reduce the jittering of the marker
    if (!n_param.getParam("use_history", useHistory_))
      useHistory_ = true;
    ROS_INFO("\tUse history: %d", useHistory_);

//     n_param.param ("marker_pattern", local_path, std::string ("data/patt.hiro"));
//     sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
//     ROS_INFO ("\tMarker Pattern Filename: %s", pattern_filename_);

	//modifications to allow patterns to be loaded from outside the package
	n_param.param ("marker_pattern", local_path, default_path);
	if (local_path.compare(0,5,"data/") == 0){
	  //to keep working on previous implementations, check if first 5 chars equal "data/"
	  sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
	}
	else{
	  //for new implementations, can pass a path outside the package_path
	  sprintf (pattern_filename_, "%s", local_path.c_str ());
	}

    n_param.param ("marker_center_x", marker_center_[0], 0.0);
    n_param.param ("marker_center_y", marker_center_[1], 0.0);
    ROS_INFO ("\tMarker Center: (%.1f,%.1f)", marker_center_[0], marker_center_[1]);

    // **** subscribe

    ROS_INFO ("Subscribing to info topic");
    sub_ = n_.subscribe (cameraInfoTopic_, 1, &ARSinglePublisher::camInfoCallback, this);
    getCamInfo_ = false;
    
    // **** advertsie 

    arMarkerPub_   = n_.advertise<ar_pose::ARMarker>("ar_pose_marker", 0);
    if(publishVisualMarkers_){ 
		rvizMarkerPub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	 }
  }

  ARSinglePublisher::~ARSinglePublisher (void)
  {
    //cvReleaseImage(&capture); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }

  void ARSinglePublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!getCamInfo_)
    {
      cam_info_ = (*cam_info);

      cam_param_.xsize = cam_info_.width;
      cam_param_.ysize = cam_info_.height;
      
      cam_param_.mat[0][0] = cam_info_.P[0];
      cam_param_.mat[1][0] = cam_info_.P[4];
      cam_param_.mat[2][0] = cam_info_.P[8];
      cam_param_.mat[0][1] = cam_info_.P[1];
      cam_param_.mat[1][1] = cam_info_.P[5];
      cam_param_.mat[2][1] = cam_info_.P[9];
      cam_param_.mat[0][2] = cam_info_.P[2];
      cam_param_.mat[1][2] = cam_info_.P[6];
      cam_param_.mat[2][2] = cam_info_.P[10];
      cam_param_.mat[0][3] = cam_info_.P[3];
      cam_param_.mat[1][3] = cam_info_.P[7];
      cam_param_.mat[2][3] = cam_info_.P[11];
     
	  cam_param_.dist_factor[0] = cam_info_.K[2];       // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = cam_info_.K[5];       // y0 = cY from openCV calibration
      if ( cam_info_.distortion_model == "plumb_bob" && cam_info_.D.size() == 5)
        cam_param_.dist_factor[2] = -100*cam_info_.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      else
        cam_param_.dist_factor[2] = 0;                  // We don't know the right value, so ignore it

      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...
	     
      arInit();

      ROS_INFO ("Subscribing to image topic");
      cam_sub_ = it_.subscribe (cameraImageTopic_, 1, &ARSinglePublisher::getTransformationCallback, this);
      getCamInfo_ = true;
    }
  }

  void ARSinglePublisher::arInit ()
  {   
    arInitCparam (&cam_param_);

    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load pattern file
    ROS_INFO ("Loading pattern");
    patt_id_ = arLoadPatt (pattern_filename_);
    if (patt_id_ < 0)
    {
      ROS_ERROR ("Pattern file load error: %s", pattern_filename_);
      ROS_BREAK ();
    }

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
#if ROS_VERSION_MINIMUM(1, 9, 0)
// FIXME: Why is this not in the object
    cv_bridge::CvImagePtr capture_;
#else
// DEPRECATED: Fuerte support ends when Hydro is released
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
#endif

  }

  void ARSinglePublisher::getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg)
  {
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k;

    /* Get the image from ROSTOPIC
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the 
     * ARToolKit configure option (see config.h).*/
#if ROS_VERSION_MINIMUM(1, 9, 0)
    try
    {
      capture_ = cv_bridge::toCvCopy (image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    dataPtr = (ARUint8 *) ((IplImage) capture_->image).imageData;
#else
    try
    {
      capture_ = bridge_.imgMsgToCv (image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException & e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    dataPtr = (ARUint8 *) capture_->imageData;
#endif

    // detect the markers in the video frame 
    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      ROS_FATAL ("arDetectMarker failed");
      ROS_BREAK ();             // FIXME: I don't think this should be fatal... -Bill
    }

    // check for known patterns
    k = -1;
    for (i = 0; i < marker_num; i++)
    {
      if (marker_info[i].id == patt_id_)
      {
        ROS_DEBUG ("Found pattern: %d ", patt_id_);

        // make sure you have the best pattern (highest confidence factor)
        if (k == -1)
          k = i;
        else if (marker_info[k].cf < marker_info[i].cf)
          k = i;
      }
    }

    if (k != -1)
    {
      // **** get the transformation between the marker and the real camera
      double arQuat[4], arPos[3];

      if (!useHistory_ || contF == 0)
        arGetTransMat (&marker_info[k], marker_center_, markerWidth_, marker_trans_);
      else
        arGetTransMatCont (&marker_info[k], marker_trans_, marker_center_, markerWidth_, marker_trans_);

      contF = 1;

      //arUtilMatInv (marker_trans_, cam_trans);
      arUtilMat2QuatPos (marker_trans_, arQuat, arPos);

      // **** convert to ROS frame

      double quat[4], pos[3];
    
      pos[0] = arPos[0] * AR_TO_ROS;
      pos[1] = arPos[1] * AR_TO_ROS;
      pos[2] = arPos[2] * AR_TO_ROS;

      quat[0] = -arQuat[0];
      quat[1] = -arQuat[1];
      quat[2] = -arQuat[2];
      quat[3] = arQuat[3];

      ROS_DEBUG (" QUAT: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
      ROS_DEBUG ("     Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);

      // **** publish the marker

		  ar_pose_marker_.header.frame_id = image_msg->header.frame_id;
		  ar_pose_marker_.header.stamp    = image_msg->header.stamp;
		  ar_pose_marker_.id              = marker_info->id;

		  ar_pose_marker_.pose.pose.position.x = pos[0];
		  ar_pose_marker_.pose.pose.position.y = pos[1];
		  ar_pose_marker_.pose.pose.position.z = pos[2];

		  ar_pose_marker_.pose.pose.orientation.x = quat[0];
		  ar_pose_marker_.pose.pose.orientation.y = quat[1];
		  ar_pose_marker_.pose.pose.orientation.z = quat[2];
		  ar_pose_marker_.pose.pose.orientation.w = quat[3];
		
		  ar_pose_marker_.confidence = marker_info->cf;

		  arMarkerPub_.publish(ar_pose_marker_);
		  ROS_DEBUG ("Published ar_single marker");
		
      // **** publish transform between camera and marker

#if ROS_VERSION_MINIMUM(1, 9, 0)
      tf::Quaternion rotation (quat[0], quat[1], quat[2], quat[3]);
      tf::Vector3 origin (pos[0], pos[1], pos[2]);
      tf::Transform t (rotation, origin);
#else
// DEPRECATED: Fuerte support ends when Hydro is released
      btQuaternion rotation (quat[0], quat[1], quat[2], quat[3]);
      btVector3 origin (pos[0], pos[1], pos[2]);
      btTransform t (rotation, origin);
#endif


      if(publishTf_)
      {
        if(reverse_transform_)
        {
          tf::StampedTransform markerToCam (t.inverse(), image_msg->header.stamp, markerFrame_.c_str(), image_msg->header.frame_id);
          broadcaster_.sendTransform(markerToCam);
        } else {
          tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, markerFrame_.c_str());
          broadcaster_.sendTransform(camToMarker);
        }
      }

      // **** publish visual marker

      if(publishVisualMarkers_)
      {
#if ROS_VERSION_MINIMUM(1, 9, 0)
        tf::Vector3 markerOrigin (0, 0, 0.25 * markerWidth_ * AR_TO_ROS);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = t * m; // marker pose in the camera frame
#else
// DEPRECATED: Fuerte support ends when Hydro is released
        btVector3 markerOrigin (0, 0, 0.25 * markerWidth_ * AR_TO_ROS);
        btTransform m (btQuaternion::getIdentity (), markerOrigin);
        btTransform markerPose = t * m; // marker pose in the camera frame
#endif

      
        tf::poseTFToMsg(markerPose, rvizMarker_.pose);

			  rvizMarker_.header.frame_id = image_msg->header.frame_id;
			  rvizMarker_.header.stamp = image_msg->header.stamp;
			  rvizMarker_.id = 1;

			  rvizMarker_.scale.x = 1.0 * markerWidth_ * AR_TO_ROS;
			  rvizMarker_.scale.y = 1.0 * markerWidth_ * AR_TO_ROS;
			  rvizMarker_.scale.z = 0.5 * markerWidth_ * AR_TO_ROS;
			  rvizMarker_.ns = "basic_shapes";
			  rvizMarker_.type = visualization_msgs::Marker::CUBE;
			  rvizMarker_.action = visualization_msgs::Marker::ADD;
			  rvizMarker_.color.r = 0.0f;
			  rvizMarker_.color.g = 1.0f;
			  rvizMarker_.color.b = 0.0f;
			  rvizMarker_.color.a = 1.0;
			  rvizMarker_.lifetime = ros::Duration(1.0);
			
			  rvizMarkerPub_.publish(rvizMarker_);
			  ROS_DEBUG ("Published visual marker");
      }
    }
    else
    {
      contF = 0;
      ROS_DEBUG ("Failed to locate marker");
    }
  }
}                               // end namespace ar_pose

