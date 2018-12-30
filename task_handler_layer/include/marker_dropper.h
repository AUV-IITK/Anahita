#pragma once

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>

class MarkerDropper {
public:
    MarkerDropper ();
    ~MarkerDropper ();
    void setActive (bool);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    
    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;

    taskHandler th;
    ros::NodeHandle nh_;
};
