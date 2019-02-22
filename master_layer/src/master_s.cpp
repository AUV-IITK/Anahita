#include <ros/ros.h>

#include <gate.h>

#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_downward_server.h>
#include <depth_stabilise.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <motion_layer/anglePIDAction.h>
#include <motion_layer/rollPIDAction.h>
#include <motion_layer/pitchPIDAction.h>
#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/upwardPIDAction.h>

#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <task_handler.h>
#include <navigation_handler.h>

#define FORWARD 1
#define BACKWARD -1

using namespace std;

void spinThread() {
    ROS_INFO("Spinning");
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1); 
    ros::Time::init();

    taskHandler th(30); // time out 15 seconds

    navigationHandler nav_handle;

    boost::thread spin_thread(&spinThread);
    
    gateTask gate_task;

    moveSideward move_sideward;
    moveStraight move_straight;
    moveDownward move_downward;
    depthStabilise depth_stabilise;

    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient("forwardPID");
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient("turnPID");

    motion_layer::sidewardPIDGoal sidewardPIDGoal;
    motion_layer::forwardPIDGoal forwardPIDGoal;
    motion_layer::upwardPIDGoal upwardPIDGoal;
    motion_layer::anglePIDGoal anglePIDGoal;

    /////////////////////////////////////////////

    // Random code to test

    /////////////////////////////////////////////


    spin_thread.join();

    return 0;
}
