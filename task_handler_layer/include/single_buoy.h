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

#include <task_handler.h>
#include <boost/thread.hpp>
#include <string>

#include <depth_stabilise.h>

#include <mutex>

class singleBuoy {
public:
    singleBuoy ();
    ~singleBuoy ();
    bool setActive (bool);
    void callback (const std_msgs::Float32Ptr&);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    motion_layer::sidewardPIDGoal sidewardPIDgoal;
    motion_layer::forwardPIDGoal forwardPIDgoal;
    motion_layer::anglePIDGoal anglePIDGoal;
    motion_layer::upwardPIDGoal upwardPIDgoal;

    taskHandler th;
    double forward_distance = 0;

    std::mutex mtx;
};
#endif // SINGLE_BUOY_H
