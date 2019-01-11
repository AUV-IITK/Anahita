#ifndef MOVE_DOWNWARD_SERVER_H
#define MOVE_DOWNWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <boost/thread.hpp>
#include <string>
#include <mutex>

class moveDownward {

protected:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;

    double angle_ = 0;
    bool angleReceived = false;
    boost::thread* spin_thread;
    boost::thread* spin_thread_;
    std::mutex mtx;

    int pwm = 0;

public:

    moveDownward(int);
    ~moveDownward();

    void setActive(bool, std::string);
    void setThrust(int);
    void spinThread();
    void spinThread_();
    void angleCB(const std_msgs::Float32ConstPtr& _msg);
};
#endif // MOVE_DOWNWARD_SERVER_H
