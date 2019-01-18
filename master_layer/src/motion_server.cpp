#include <ros/ros.h>
#include <angle_PID_server.h>
#include <forward_PID_server.h>
#include <sideward_PID_server.h>
#include <upward_PID_server.h>
#include <roll_PID_server.h>
#include <pitch_PID_server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion");
    ROS_INFO("Motion Server Activated");

    ros::Time::init();
    
    forwardPIDAction forward_motion("forwardPID");
    sidewardPIDAction sideward_motion("sidewardPID");
    upwardPIDAction upward_motion("upwardPID");
    anglePIDAction turn_motion("turnPID");
    pitchPIDAction pitch_motion("pitchPID");
    rollPIDAction roll_motion("rollPID"); 
    
    ros::spin();
    return 0;
}
