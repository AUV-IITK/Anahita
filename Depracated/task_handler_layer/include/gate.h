#ifndef GATE_H
#define GATE_H

#include <ros/ros.h>

#include <string>

#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>

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

    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;

    motion_layer::swayPIDGoal swayPIDgoal;
    motion_layer::yawPIDGoal yawPIDGoal;

    taskHandler th;
    depthStabilise depth_stabilise;

    std::mutex mtx;

public:
    gateTask();
    ~gateTask();

    bool setActive(bool);
};

#endif // GATE_H
