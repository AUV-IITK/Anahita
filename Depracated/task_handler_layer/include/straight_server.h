#ifndef STRAIGHT_SERVER_H
#define STRAIGHT_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/yawPIDAction.h>
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
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;    
    motion_layer::yawPIDGoal yaw_PID_goal;

    ros::Subscriber sub_;

    boost::thread* spin_thread;

public:

    moveStraight();
    ~moveStraight();

    void activate (int, std::string);
    void deActivate ();

    void setThrust(int);
    void spinThread();
};
#endif // STRAIGHT_SERVER_H
