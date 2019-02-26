#ifndef YAW_PID_SERVER_H
#define YAW_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/yawPIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

class yawPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::yawPIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::yawPIDFeedback feedback_;
    motion_layer::yawPIDResult result_;
    double goal_;
    double current_yaw_ = 0;
    bool goalReceived = false;
    ros::Subscriber sub_;
    ErrorDescriptor yaw;

    boost::thread* spin_thread;

public:

    yawPIDAction(std::string);
    ~yawPIDAction();
    void goalCB();
    void preemptCB();
    void callBack(const std_msgs::Float32ConstPtr&);
};
#endif // YAW_PID_SERVER_H
