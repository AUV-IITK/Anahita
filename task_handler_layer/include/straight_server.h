#ifndef STRAIGHT_SERVER_H
#define STRAIGHT_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32.h>
#include <string>

#include <iostream>
#include <vector>
#include <map>

class moveStraight {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;

    bool goalReceived;
    bool& goalReceived_ref = goalReceived;
    double angle;
    ros::Subscriber sub_;

    boost::thread* spin_thread;
    boost::thread* spin_thread_;

public:

    moveStraight(int);
    ~moveStraight();

    void setActive(bool);
    void setThrust(int);
    void spinThread_();
    void imuAngleCB(const std_msgs::Float32Ptr &_msg);
    void spinThread();
};
#endif // STRAIGHT_SERVER_H
