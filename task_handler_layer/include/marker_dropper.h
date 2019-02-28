#pragma once

#include <ros/ros.h>

#include <motion_layer/surgePIDAction.h>
#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>

#include <straight_server.h>
#include <depth_stabilise.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>
#include <mutex>

class MarkerDropper {
public:
    MarkerDropper ();
    ~MarkerDropper ();
    bool setActive (bool);

private:
    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;

    motion_layer::yawPIDGoal yawPIDGoal;    
    motion_layer::swayPIDGoal swayPIDGoal;
    motion_layer::surgePIDGoal surgePIDgoal;

    taskHandler th;
    ros::NodeHandle nh;
    ros::Subscriber surge_sub_;
    moveStraight move_straight;
    depthStabilise depth_stabilise;

};
