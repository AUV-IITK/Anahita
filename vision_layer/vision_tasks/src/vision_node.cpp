#include "buoy.h"
#include "gate.h"
#include "markerDropper.h"
#include "octagon.h"
#include "torpedo.h"

#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

std::string current_task = "red_buoy";
std::string previous_task = "";

void taskCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Task being chaned to: [%s]", msg->data.c_str());
    current_task = msg->data;
}

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node");    
    ros::NodeHandle nh;

    Buoy buoy;
    Gate gate;
    Torpedo torpedo;
    MarkerDropper md;
    Octagon octagon;

    ros::Subscriber current_task_sub = nh.subscribe<std_msgs::String>("/current_task", 1000,taskCallback);

    ros::Rate loop_rate(10);

    ROS_INFO("Vision Node started");

    while (ros::ok()) {
        if (current_task != previous_task) {
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
                gate.frontTaskHandling(true);
            }
            if (previous_task == "gate") {
                gate.frontTaskHandling(false);
            }
            if (current_task == "red_torpedo") {
                torpedo.switchColor(1);
                torpedo.TaskHandling(true);
            }
            if (current_task == "green_torpedo") {
                torpedo.switchColor(0);
            }
            if (previous_task == "green_torpedo") {
                torpedo.TaskHandling(false);     
            }
            if (current_task == "marker_dropper_front") {
                md.FrontTaskHandling(true);
            }
            if (previous_task == "marker_dropper_front") {
                md.FrontTaskHandling(false);
            }
            if (current_task == "marker_dropper_bottom") {
                md.BottomTaskHandling(true);
            }
            if (previous_task == "marker_dropper_bottom") {
                md.BottomTaskHandling(false);
            }
            if (current_task == "octagon") {
                octagon.BottomTaskHandling(true);
            }
            if (previous_task == "octagon") {
                octagon.BottomTaskHandling(false);
            }
            previous_task = current_task;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
