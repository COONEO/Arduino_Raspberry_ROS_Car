#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "launch_file_node");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}