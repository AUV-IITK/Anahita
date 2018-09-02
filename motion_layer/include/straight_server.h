#ifndef STRAIGHT_SERVER_H
#define STRAIGHT_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include <limits>

class moveStraight {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;
    boost::thread* spin_thread;
    double angle;
    ros::Subscriber sub_;

public:

    moveStraight(int);
    ~moveStraight();

    void setActive(bool);
    void spinThread();
    void imuAngleCB(const std_msgs::Float64Ptr &_msg);
};
#endif // STRAIGHT_SERVER_H
