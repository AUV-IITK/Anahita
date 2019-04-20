#pragma once

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <motion_layer/heavePIDAction.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <string>

#include <iostream>
#include <vector>
#include <map>
#include <mutex>

#include <straight_server.h>

class depthStabilise {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient;    
    
    motion_layer::heavePIDGoal heave_PID_goal;

    boost::thread* spin_thread;
    moveStraight move_straight;

public:

    depthStabilise();
    ~depthStabilise();

    void activate (std::string);
    void deActivate ();
    void spinThread();
};