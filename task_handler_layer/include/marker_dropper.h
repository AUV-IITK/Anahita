#pragma once

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>

#include <straight_server.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>
#include <mutex>

#include <line.h>

class MarkerDropper {
public:
    MarkerDropper ();
    ~MarkerDropper ();
    bool setActive (bool);
    void forwardCB (const std_msgs::Float32ConstPtr &_msg);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;

    motion_layer::anglePIDGoal anglePIDGoal;    
    motion_layer::sidewardPIDGoal sidewardPIDGoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;

    taskHandler th;
    ros::NodeHandle nh;
    ros::Subscriber forward_sub_;
    std::mutex mtx;
    moveStraight move_straight;

    bool forwardGoalReceived = false;
    double forward_distance_ = 0;
};
