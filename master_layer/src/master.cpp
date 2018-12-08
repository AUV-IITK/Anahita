#include <ros/ros.h>
#include <buoy.h>
#include <line.h>
#include <gate.h>
#include <single_buoy.h>
#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_forward_server.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;

    // singleBuoy single_buoy;
    // single_buoy.setActive(true);

    // gateTask gate_task;
    // gate_task.setActive(true);
    // cout << "Start" << endl;

    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient("turnPID");    
    motion_layer::anglePIDGoal angle_PID_goal;

    ROS_INFO("Waiting for anglePID server to start.");
    anglePIDClient.waitForServer();

    ROS_INFO("anglePID server started, sending goal.");
    angle_PID_goal.target_angle = 179;
    anglePIDClient.sendGoal(angle_PID_goal);

    // moveStraight move_straight_(100);
    // move_straight_.setActive(true);
    // ros::Duration(20).sleep();
    // move_straight_.setActive(false);

    // actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient("forwardPID");    
    // motion_layer::forwardPIDGoal forward_PID_goal;

    // ROS_INFO("Waiting for forwardPID server to start.");
    // forwardPIDClient.waitForServer();

    // ROS_INFO("forwardPID server started, sending goal.");
    // forward_PID_goal.target_distance = 275;
    // forwardPIDClient.sendGoal(forward_PID_goal);

    // actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");    
    // motion_layer::upwardPIDGoal upward_PID_goal;

    // ROS_INFO("Waiting for upwardPID server to start.");
    // upwardPIDClient.waitForServer();

    // ROS_INFO("upwardPID server started, sending goal.");
    // upward_PID_goal.target_depth = 0;
    // upwardPIDClient.sendGoal(upward_PID_goal);

    // actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");    
    // motion_layer::sidewardPIDGoal sideward_PID_goal;

    // ROS_INFO("Waiting for sidewardPID server to start.");
    // sidewardPIDClient.waitForServer();

    // ROS_INFO("sidewardPID server started, sending goal.");
    // sideward_PID_goal.target_distance = 0;
    // sidewardPIDClient.sendGoal(sideward_PID_goal);

    // moveForward move_forward_(150);
    // move_forward_.setActive(true);
    // ros::Duration(10).sleep();
    // move_forward_.setActive(false);

    // moveSideward move_sideward_(100);
    // move_sideward_.setActive(true);
    // ros::Duration(20).sleep();
    // move_sideward_.setActive(false);

    //singleBuoy single_buoy_;
    //single_buoy_.setActive(true);

    // ros::spin();

    return 0;
}
