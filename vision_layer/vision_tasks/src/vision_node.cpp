#include "buoy.h"
#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "vision_node");    
    Buoy buoy_;
    buoy_.TaskHandling();
}