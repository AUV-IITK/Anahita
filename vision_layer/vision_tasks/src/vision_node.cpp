#include "gate.h"
#include "path_marker.h"
#include "marker.h"
#include "torpedo.h"
#include "crucifix.h"
#include "grabber.h"
#include "testgate.h"
#include "triangular_buoy.h"
#include "start_gate.h"
#include "buoy.h"

#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

#include <master_layer/CurrentTask.h>
#include <std_msgs/String.h>

std::string current_task = "torpedo";
std::string previous_task = "";

bool changeCurrentTask(master_layer::CurrentTask::Request &req,
                       master_layer::CurrentTask::Response &res)
{
    current_task = req.current_task;
    res.status = true;
    return true;
}

void currentTaskCB (std_msgs::String msg) {
    current_task = msg.data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    ros::Time::init();
    ros::ServiceServer service = nh.advertiseService("anahita/current_task", changeCurrentTask);
    ros::Subscriber current_task_sub = nh.subscribe("anahita/current_task", 1, currentTaskCB);

    ros::Duration(1).sleep();

    Gate gate;
    Torpedo torpedo;
    PathMarker path_marker;
    Marker marker;
    Crucifix crucifix;
    Grabber grabber;
    TestGate testgate;
    TriangularBuoy triangular_buoy;
    StartGate start_gate;
    Buoy buoy;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (current_task != previous_task)
        {
            if(current_task == "grabber_test")
            {
                ROS_INFO("grabber_test");
                grabber.frontTaskHandling(true);
            }
            if(previous_task == "grabber_test")
            {
                grabber.frontTaskHandling(false);
            }
            if(current_task == "crucifix")
            {
                ROS_INFO("crucfix_task");
                crucifix.frontTaskHandling(true);
            }
            if(previous_task == "crucifix")
            {
                crucifix.frontTaskHandling(false);
            }
            if (current_task == "gate")
            {
                ROS_INFO("gate_task");
                gate.frontTaskHandling(true);
            }
            if (previous_task == "gate")
            {
                gate.frontTaskHandling(false);
            }
            if (current_task == "marker") {
                ROS_INFO("marker task");
                marker.frontTaskHandling(true);
            }
            if (previous_task == "marker") {
                marker.frontTaskHandling(false);
            }
            if (current_task == "torpedo") {
                ROS_INFO("torpedo task");
                torpedo.frontTaskHandling(true);
            }
            if (previous_task == "torpedo") {
                torpedo.frontTaskHandling(false);
            }
            if (current_task == "testgate") {
                testgate.frontTaskHandling(true);
            }
            if (previous_task == "testgate") {
                testgate.frontTaskHandling(false);
            }
            if (current_task == "triangular_buoy") {
                triangular_buoy.frontTaskHandling(true);
            }
            if (previous_task == "triangular_buoy") {
                triangular_buoy.frontTaskHandling(false);
            }
            if (current_task == "path_marker") {
                path_marker.bottomTaskHandling(true); 
            }
            if (previous_task == "path_marker") {
                path_marker.bottomTaskHandling(false);
            }
            if (current_task == "start_gate") {
                ROS_INFO ("Start gate");
                start_gate.frontTaskHandling(true); 
            }
            if (previous_task == "start_gate") {
                start_gate.frontTaskHandling(false);
            }
            if (current_task == "buoy") {
                ROS_INFO ("Buoy");
                buoy.frontTaskHandling(true); 
            }
            if (previous_task == "buoy") {
                buoy.frontTaskHandling(false);
            }

            previous_task = current_task;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
