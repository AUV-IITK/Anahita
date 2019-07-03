#include <ros/ros.h>
#include "mapping_main.h"
#include "slam_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_node");

  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  mapping::MappingNode mapping_node(nh);
  mapping::SlamNode node(nh);
  node.add_landmark();
  ROS_INFO("Started the mapping_node handler");
  mapping_node.Spin();
  return 0;

}