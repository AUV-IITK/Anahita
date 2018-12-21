#include "buoy.h"
#include "torpedo.h"
#include "line.h"
#include "markerDropper.h"
#include "gate.h"
#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

std::string current_task, previous_task;

Buoy buoy;
Gate gate;
Line line;
MarkerDropper md;
Torpedo torpedo;

void taskHandler() {
    if (current_task == "red_buoy") {
        buoy.switchColor(0);
        buoy.TaskHandling(true);
    }
    if (current_task == "yellow_buoy") {
        buoy.switchColor(1);
    }
    if (current_task == "green_buoy") {
        buoy.switchColor(2);
    }
    if (previous_task == "green_buoy") {
        buoy.TaskHandling(false);
    }
    if (current_task == "gate") {
        gate.TaskHandling(true);
    }
    if (previous_task == "gate") {
        gate.TaskHandling(false);
    }
    previous_task = current_task;
}

void taskCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Task being chaned to: [%s]", msg->data.c_str());
    current_task = msg->data;
    taskHandler();
}

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node");    
    ros::NodeHandle nh;
    ros::Subscriber current_task_sub = nh.subscribe<std_msgs::String>("/current_task", 1000,taskCallback);

    ros::spin();

    return 0;
}
