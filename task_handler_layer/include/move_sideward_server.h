#ifndef MOVE_SIDEWARD_SERVER_H
#define MOVE_SIDEWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <string>

class moveSideward {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;
    ros::Subscriber sub_;
    double angle;
    bool goalReceived;

public:

    moveSideward(int);
    ~moveSideward();

    boost::thread* spin_thread;
    boost::thread* spin_thread_;
    void setActive(bool);
    void setThrust(int);
    void imuAngleCB(const std_msgs::Float32Ptr &_msg);
    void spinThread();
    void spinThread_();
};
#endif // MOVE_SIDEWARD_SERVER_H
