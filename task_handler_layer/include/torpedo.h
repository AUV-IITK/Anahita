#ifndef TORPEDO_H
#define TORPEDO_H

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>

class Torpedo {
public:
    Torpedo();
    ~Torpedo();
    bool setActive(bool);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;
    
    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;
    motion_layer::anglePIDGoal anglePIDGoal;
    motion_layer::upwardPIDGoal upwardPIDgoal;

    taskHandler th;

    ros::NodeHandle nh_;
};
#endif // TORPEDO_H
