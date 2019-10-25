// ROS and node class header file

#include "pcl_tutorial.hpp"


int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "pcl_tutorial");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  pcl_tutorial::My_pcl node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
