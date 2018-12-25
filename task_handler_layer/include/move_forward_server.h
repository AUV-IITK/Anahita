#ifndef MOVE_FORWARD_SERVER_H
#define MOVE_FORWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>
#include <boost/thread.hpp>

class moveForward {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient_;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient_;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient_;
    ros::Subscriber angle_sub_;
    ros::Subscriber depth_sub_;

    motion_layer::sidewardPIDGoal sideward_PID_goal;
    motion_layer::upwardPIDGoal upward_PID_goal;
    motion_layer::anglePIDGoal angle_PID_goal;

    double angle;
    double depth;
    std::string upward_type_;
    std::string angle_type_;

    bool depthGoalReceived;
    bool angleGoalReceived;

    boost::thread* spin_thread;
    boost::thread* spin_thread_;

public:

    moveForward(int pwm_);
    ~moveForward();

    void setActive(bool);
    void setThrust(int);
    void setReferenceAngle(double);
    void setReferenceDepth(double);
    void imuAngleCB(const std_msgs::Float32Ptr &_msg);
    void depthCB(const std_msgs::Float32Ptr &_msg);
    void spinThread();
    void spinThread_();
};
#endif // MOVE_FORWARD_SERVER_H
