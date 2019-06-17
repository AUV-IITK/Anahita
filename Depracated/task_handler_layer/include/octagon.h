#pragma once

#include <ros/ros.h>

#include <motion_layer/surgePIDAction.h>
#include <motion_layer/swayPIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>

class Octagon {
public:
    Octagon ();
    ~Octagon ();
    bool setActive (bool);
    void spinThread ();

private:
    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    
    motion_layer::swayPIDGoal swayPIDgoal;
    motion_layer::surgePIDGoal surgePIDgoal;

    taskHandler th;
    
    ros::NodeHandle nh_;
};
