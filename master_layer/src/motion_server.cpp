#include <ros/ros.h>
#include <yaw_PID_server.h>
#include <surge_PID_server.h>
#include <sway_PID_server.h>
#include <heave_PID_server.h>
#include <roll_PID_server.h>
#include <pitch_PID_server.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion");
    ROS_INFO("Motion Server Activated");

    ros::Time::init();
    
    surgePIDAction surge_motion("surgePID");
    swayPIDAction sway_motion("swayPID");
    heavePIDAction heave_motion("heavePID");
    yawPIDAction turn_motion("yawPID");
    pitchPIDAction pitch_motion("pitchPID");
    rollPIDAction roll_motion("rollPID"); 
    
    ros::spin();
    return 0;
}
