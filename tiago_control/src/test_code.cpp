#include "ros/ros.h"

int main(int argc, char** argv) 
{
  ros::init(argc,argv,"cpp_node"); // initialize the ROS node
  ros::NodeHandle nh; // ros handle
  ros::Rate loop_rate(2);

  while(ros::ok())
  {
    ROS_INFO("Hello, C++ Node v1");
  }

  return 0;
}