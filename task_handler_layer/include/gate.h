#ifndef GATE_H
#define GATE_H

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include <boost/thread.hpp>
#include <string>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

class gateTask
{
private:
    ros::NodeHandle nh_;
    boost::thread* spin_thread;

    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;

    ros::Subscriber forward_sub_;
    ros::Subscriber sideward_sub_;
    ros::Subscriber angle_sub_;

    bool angleGoalReceived;

    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;
    motion_layer::anglePIDGoal anglePIDGoal;

    double forward_distance_;
    double sideward_distance_;
    double angle_;

public:
    gateTask();
    ~gateTask();

    void setActive(bool);
    void spinThread();
    void forwardCB(const std_msgs::Float32Ptr &_msg);
    void sidewardCB(const std_msgs::Float32Ptr &_msg);
    void angleCB(const std_msgs::Float32Ptr &_msg);
};

#endif // GATE_H