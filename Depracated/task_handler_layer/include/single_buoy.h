#ifndef SINGLE_BUOY_H
#define SINGLE_BUOY_H

#include <ros/ros.h>

#include <motion_layer/surgePIDAction.h>
#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>
#include <motion_layer/heavePIDAction.h>

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
    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    motion_layer::swayPIDGoal swayPIDgoal;
    motion_layer::surgePIDGoal surgePIDgoal;
    motion_layer::yawPIDGoal yawPIDGoal;
    motion_layer::heavePIDGoal heavePIDgoal;

    taskHandler th;
    double forward_distance = 0;

    bool goal_received = false;

    std::mutex mtx;
};
#endif // SINGLE_BUOY_H
