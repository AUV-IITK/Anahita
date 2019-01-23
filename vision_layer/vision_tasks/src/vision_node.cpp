#include "buoy.h"
#include "gate.h"
#include "markerDropper.h"
#include "octagon.h"
#include "torpedo.h"
#include "line.h"

#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

std::string current_task = "red_buoy";
std::string previous_task = "";

// void taskCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("Task being chaned to: [%s]", msg->data.c_str());
//     current_task = msg->data;
// }

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node");    
    ros::NodeHandle nh;

    Buoy buoy;
    Gate gate;
    Torpedo torpedo;
    MarkerDropper md;
    Octagon octagon;
    Line line;

    // ros::Subscriber current_task_sub = nh.subscribe<std_msgs::String>("/current_task", 1000,taskCallback);

    ros::Rate loop_rate(10);

    ROS_INFO("Vision Node started");

    while (ros::ok()) {
        nh.getParam("/current_task", current_task);
        if (current_task != previous_task) {

            system("rosparam delete /vision_node");

            if (current_task == "red_buoy") {
                buoy.switchColor(0);
                if (!buoy.task_on)
                    buoy.TaskHandling(true);
            }
            if (current_task == "yellow_buoy") {
                buoy.switchColor(1);
                if (!buoy.task_on)
                    buoy.TaskHandling(true);
            }
            if (current_task == "green_buoy") {
                buoy.switchColor(2);
                if (!buoy.task_on)
                    buoy.TaskHandling(true);
            }
            if (previous_task == "yellow_buoy" || previous_task == "red_buoy" || previous_task == "green_buoy") {
                if (current_task == "red_buoy" || current_task == "yellow_buoy" || current_task == "green_buoy") {}
                else { buoy.TaskHandling(false); }
            }
            if (current_task == "gate_front") {
                gate.frontTaskHandling(true);
            }
            if (previous_task == "gate_front") {
                gate.frontTaskHandling(false);
            }
            if (current_task == "gate_bottom") {
                gate.bottomTaskHandling(true);
            }
            if (previous_task == "gate_bottom") {
                gate.bottomTaskHandling(false);
            }
            if (current_task == "green_torpedo") {
                torpedo.switchColor(0);
                if (!torpedo.task_on)
                    torpedo.TaskHandling(true);
            }
            if (current_task == "red_torpedo") {
                torpedo.switchColor(1);
                if (!torpedo.task_on)
                    torpedo.TaskHandling(true);
            }
            if (previous_task == "red_torpedo" || previous_task == "green_torpedo") {
                if (current_task == "red_torpedo" || current_task == "green_torpedo") {}
                else { torpedo.TaskHandling(false); }     
            }
            if (current_task == "marker_dropper_front") {
                md.frontTaskHandling(true);
            }
            if (previous_task == "marker_dropper_front") {
                md.frontTaskHandling(false);
            }
            if (current_task == "marker_dropper_bottom") {
                md.bottomTaskHandling(true);
            }
            if (previous_task == "marker_dropper_bottom") {
                md.bottomTaskHandling(false);
            }
            if (current_task == "line") {
                line.TaskHandling(true);
            }
            if (previous_task == "line") {
                line.TaskHandling(false);
            }
            if (current_task == "octagon") {
                octagon.bottomTaskHandling(true);
            }
            if (previous_task == "octagon") {
                octagon.bottomTaskHandling(false);
            }
            previous_task = current_task;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

