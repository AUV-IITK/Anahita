#ifndef TORPEDO_H
#define TORPEDO_H

#include <ros/ros.h>

#include <motion_layer/surgePIDAction.h>
#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>
#include <motion_layer/heavePIDAction.h>

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
    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient;
    
    motion_layer::swayPIDGoal swayPIDgoal;
    motion_layer::surgePIDGoal surgePIDgoal;
    motion_layer::yawPIDGoal yawPIDGoal;
    motion_layer::heavePIDGoal heavePIDgoal;

    taskHandler th;

    ros::NodeHandle nh_;
};
#endif // TORPEDO_H
