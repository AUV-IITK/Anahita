#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <actionlib/client/terminal_state.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32.h>
#include <string>

#include <iostream>
#include <vector>
#include <map>

class depthStabilise {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;    
    
    motion_layer::anglePIDGoal angle_PID_goal;
    motion_layer::upwardPIDGoal upward_PID_goal;

    bool goalReceived;
    bool close_loop = false;
    bool& goalReceived_ref = goalReceived;
    double depth;
    ros::Subscriber sub_;

    boost::thread* spin_thread;

public:

    depthStabilise();
    ~depthStabilise();

    void setActive(bool);
    void depthCB(const std_msgs::Float32Ptr &_msg);
    void spinThread();
};