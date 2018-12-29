#ifndef ROLL_PID_SERVER_H
#define ROLL_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/rollPIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

class rollPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::rollPIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::rollPIDFeedback feedback_;
    motion_layer::rollPIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor roll;

    boost::thread* spin_thread;

public:

    rollPIDAction(std::string);
    ~rollPIDAction();
    void goalCB();
    void preemptCB();
    void callBack(const std_msgs::Float32ConstPtr&);
};
#endif // ROLL_PID_SERVER_H
