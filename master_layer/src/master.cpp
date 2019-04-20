#include <ros/ros.h>

#include <gate.h>

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

#include <master_layer/CurrentTask.h>

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

    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient("surgePID");
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient("heavePID");
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient("swayPID");
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient("yawPID");
    actionlib::SimpleActionClient<motion_layer::rollPIDAction> rollPIDClient("rollPID");
    actionlib::SimpleActionClient<motion_layer::pitchPIDAction> pitchPIDClient("pitchPID");

    motion_layer::swayPIDGoal swayPIDGoal;
    motion_layer::surgePIDGoal surgePIDGoal;
    motion_layer::heavePIDGoal heavePIDGoal;
    motion_layer::yawPIDGoal yawPIDGoal;
    motion_layer::rollPIDGoal rollPIDGoal;
    motion_layer::pitchPIDGoal pitchPIDGoal;

    ros::ServiceClient client = nh.serviceClient<master_layer::CurrentTask>("current_task");
    master_layer::CurrentTask currentTask_srv;

    /////////////////////////////////////////////

    /////////////////////////////////////////////

    spin_thread.join();

    return 0;
}
