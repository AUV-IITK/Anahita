#ifndef GATE_H
#define GATE_H

#include <ros/ros.h>

#include <string>

#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>
#include <navigation_handler.h>
#include <depth_stabilise.h>

#include <mutex>

class gateTask
{
private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;

    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::anglePIDGoal anglePIDGoal;

    taskHandler th;
    depthStabilise depth_stabilise;

    std::mutex mtx;

public:
    gateTask();
    ~gateTask();

    bool setActive(bool);
    void forwardCB (const std_msgs::Float32ConstPtr &_msg);
};

#endif // GATE_H
