#ifndef SINGLE_BUOY_H
#define SINGLE_BUOY_H

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>

class singleBuoy {
public:
    singleBuoy();
    ~singleBuoy();
    void setActive(bool);
    void forwardCB(const std_msgs::Float32ConstPtr &_msg);
    void sidewardCB(const std_msgs::Float32ConstPtr &_msg);
    void angleCB(const std_msgs::Float32Ptr &_msg);
    void upwardCB(const std_msgs::Float32Ptr &_msg);
    void spinThread();

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;
    
    ros::Subscriber forward_sub_;
    ros::Subscriber sideward_sub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber upward_sub_;

    bool angleGoalReceived = false;
    bool forwardGoalReceived = false;

    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;
    motion_layer::anglePIDGoal anglePIDGoal;
    motion_layer::upwardPIDGoal upwardPIDgoal;

    taskHandler th;

    boost::thread* spin_thread;

    ros::NodeHandle nh_;
    double forward_distance_;
    double sideward_distance_;
    double depth_;
    double angle_;
};
#endif // SINGLE_BUOY_H
