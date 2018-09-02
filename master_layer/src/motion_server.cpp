#include <ros/ros.h>
#include <angle_PID_server.h>
#include <forward_PID_server.h>
#include <sideward_PID_server.h>
#include <upward_PID_server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion");

    forwardPIDAction forward_motion("forwardPID");
    sidewardPIDAction sideward_motion("sidewardPID");
    // upwardPIDAction upward_motion_vision("upwardPID/vision", "VISION");
    // upwardPIDAction upward_motion_sensor("upwardPID/sensor", "SENSOR");
    anglePIDAction turn_motion_vision("turnPID");
    // anglePIDAction turn_motion_sensor("turnPID/sensor", "SENSOR");

    ros::spin();
    return 0;
}