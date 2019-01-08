#ifndef STRAIGHT_SERVER_H
#define STRAIGHT_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32.h>
#include <string>

#include <iostream>
#include <vector>
#include <map>
#include <mutex>

class moveStraight {

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;

    ros::Subscriber sub_;

    boost::thread* spin_thread;
    boost::thread* spin_thread_;

    bool angleReceived = false;
    double angle_;

    std::mutex mtx;

public:

    moveStraight(int);
    ~moveStraight();

    void setActive(bool, std::string);
    void setThrust(int);
    void spinThread();
    void spinThread_();
    void angleCB(const std_msgs::Float32ConstPtr& _msg);
};
#endif // STRAIGHT_SERVER_H
