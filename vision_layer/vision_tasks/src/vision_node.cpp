#include "buoy.h"
#include "gate.h"
#include "markerDropper.h"
// #include "octagon.h"
#include "torpedo.h"
#include "line.h"
#include "base_class.h"
#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

#include <master_layer/CurrentTask.h>

std::string current_task = "red_buoy";
std::string previous_task = "";

bool changeCurrentTask (master_layer::CurrentTask::Request &req,
                        master_layer::CurrentTask::Response &res) {
    current_task = req.current_task;
    res.status = true;
    return true;
}

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node");    
    ros::NodeHandle nh;

    Buoy buoy;
    Gate gate;
    Torpedo torpedo;
    MarkerDropper md;
    // Octagon octagon;
    Line line;
    ros::ServiceServer service = nh.advertiseService("current_task", changeCurrentTask);

    ros::Rate loop_rate(10);

    ROS_INFO("Vision Node started");

    while (ros::ok()) {
        if (current_task != previous_task) {
            if (current_task == "red_buoy") {
                buoy.switchColor(0);
                buoy.frontTaskHandling(true);
            }
            if (current_task == "yellow_buoy") {
                buoy.switchColor(1);
            }
            if (current_task == "green_buoy") {
                buoy.switchColor(2);
            }
            if (previous_task == "green_buoy") {
                buoy.frontTaskHandling(false);
            }
            if (current_task == "gate") {
                ROS_INFO("gate task");
                gate.frontTaskHandling(true);
            }
            if (previous_task == "gate") {
                gate.frontTaskHandling(false);
            }
            if (current_task == "green_torpedo") {
                torpedo.switchColor(0);
                torpedo.frontTaskHandling(true);
            }
            if (current_task == "red_torpedo") {
                torpedo.switchColor(1);
            }
            if (previous_task == "green_torpedo") {
                torpedo.frontTaskHandling(false);     
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
                ROS_INFO("Line Task Running");
                line.bottomTaskHandling(true);
            }
            if (previous_task == "line") {
                line.bottomTaskHandling(false);
            }
            // if (current_task == "octagon") {
            //     octagon.bottomTaskHandling(true);
            // }
            // if (previous_task == "octagon") {
            //     octagon.bottomTaskHandling(false);
            // }
            previous_task = current_task;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
