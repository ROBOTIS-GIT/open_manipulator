#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
