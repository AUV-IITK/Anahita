#ifndef ANGLE_PID_SERVER_H
#define ANGLE_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/anglePIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

class anglePIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::anglePIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::anglePIDFeedback feedback_;
    motion_layer::anglePIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ErrorDescriptor angle;

    boost::thread* spin_thread;

public:

    anglePIDAction(std::string);
    ~anglePIDAction();
    void goalCB();
    void preemptCB();
    void callBack(const std_msgs::Float32ConstPtr&);
};
#endif // ANGLE_PID_SERVER_H
