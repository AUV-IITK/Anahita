#include <ros/ros.h>
#include "slam_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    mapping::SlamFilter* filter=new mapping::SlamFilter(1000);
    mapping::SlamServer server(nh, filter);
    ros::ServiceServer sub=nh->advertiseService("/mapping_node/observe_landmark",&mapping::SlamServer::observe,&server);    

    
    ROS_INFO("started the slam server");

    ros::spin();
}