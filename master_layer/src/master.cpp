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

// double angle = 0;

// void imuAngleCB(const std_msgs::Float64Ptr &_msg) {
//     angle = _msg->data;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    // nh.setParam("/current_task", "LINE");
    // lineTask line_;
    // line_.setActive(true);
    // ros::Duration(5).sleep();
    // line_.setActive(false);
    // buoy buoy_;
    // nh.setParam("/current_task", "BUOY");
    // buoy_.setActive(true);
    // // TRANSITION FROM BUOY TASK TO GATE TASK //
    // nh.setParam("/current_task", "GATE");
    // gateTask gate_;
    // gate_.setActive(true);
    // ros::Duration(5).sleep();
    // TRANSITION FROM GATE TASK TO TORPEDO TASK //

    // ros::Subscriber sub_;
    // sub_ = nh.subscribe("/varun/sensors/yaw", 1, &imuAngleCB);
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient("turnPID");    
    motion_layer::anglePIDGoal angle_PID_goal;

    ROS_INFO("Waiting for anglePID server to start.");
    anglePIDClient.waitForServer();

    ROS_INFO("anglePID server started, sending goal.");
    angle_PID_goal.target_angle = 90;
    anglePIDClient.sendGoal(angle_PID_goal);

    //moveStraight move_straight_(100);
    //move_straight_.setActive(true);
    //ros::Duration(10).sleep();
    //move_straight_.setActive(false);

    //actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient("forwardPID");    
    //motion_layer::forwardPIDGoal forward_PID_goal;

    //ROS_INFO("Waiting for forwardPID server to start.");
    //forwardPIDClient.waitForServer();

    //ROS_INFO("forwardPID server started, sending goal.");
    //forward_PID_goal.target_distance = 100;
    //forwardPIDClient.sendGoal(forward_PID_goal);

    //actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");    
    //motion_layer::sidewardPIDGoal sideward_PID_goal;

    //ROS_INFO("Waiting for sidewardPID server to start.");
    //sidewardPIDClient.waitForServer();

    //ROS_INFO("sidewardPID server started, sending goal.");
    //sideward_PID_goal.target_distance = 0;
    //sidewardPIDClient.sendGoal(sideward_PID_goal);

    //moveForward move_forward_(150);
    //move_forward_.setActive(true);
    //ros::Duration(10).sleep();
    //move_forward_.setActive(false);

    //moveSideward move_sideward_(100);
    //move_sideward_.setActive(true);
    //ros::Duration(10).sleep();
    //move_sideward_.setActive(false);

    //singleBuoy single_buoy_;
    //single_buoy_.setActive(true);

    ros::spin();
    return 0;
}
