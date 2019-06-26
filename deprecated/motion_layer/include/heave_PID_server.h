#ifndef HEAVE_PID_SERVER_H
#define HEAVE_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/heavePIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string>

class heavePIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::heavePIDAction> as_; // NodeHandle instance must be created before this line. 
                                                                         // Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::heavePIDFeedback feedback_;
    motion_layer::heavePIDResult result_;
    double goal_;

    double current_heave_ = 0;
    bool goalReceived = false;

    ros::Subscriber sub_;
    ErrorDescriptor heave;

public:

    heavePIDAction(std::string);
    ~heavePIDAction();
    void goalCB();
    void preemptCB();
    void callback(const std_msgs::Float32ConstPtr&);

};
#endif 
