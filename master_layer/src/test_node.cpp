#include <ros/ros.h>

#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_downward_server.h>
#include <depth_stabilise.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <motion_layer/yawPIDAction.h>
#include <motion_layer/rollPIDAction.h>
#include <motion_layer/pitchPIDAction.h>
#include <motion_layer/surgePIDAction.h>
#include <motion_layer/swayPIDAction.h>
#include <motion_layer/heavePIDAction.h>

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
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1); 
    ros::Time::init();

    taskHandler th(30); // time out 15 seconds

    navigationHandler nav_handle;

    boost::thread spin_thread(&spinThread);
    
    moveSideward move_sideward;
    moveStraight move_straight;
    moveDownward move_downward;
    depthStabilise depth_stabilise;

    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient("surgePID");
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient("heavePID");
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient("swayPID");
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient("yawPID");

    motion_layer::swayPIDGoal swayPIDGoal;
    motion_layer::surgePIDGoal surgePIDGoal;
    motion_layer::heavePIDGoal heavePIDGoal;
    motion_layer::yawPIDGoal yawPIDGoal;

    /////////////////////////////////////////////

    // Random code to test

    move_straight.activate(50, "current");

    ros::Duration(30).sleep();

    move_straight.deActivate();

    /////////////////////////////////////////////

    spin_thread.join();

    return 0;
}
