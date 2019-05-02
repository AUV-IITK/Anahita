#include <ros/ros.h>
#include "dvl_connection.h"

int main(int argc, char *argv[])
{
    ROS_INFO("Started ROS Node");    
    ros::init(argc, argv, "dvl_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    hardware_dvl::DVLNode dvl_node(nh);
    ROS_INFO("Reached here");    

    dvl_node.Spin();
}