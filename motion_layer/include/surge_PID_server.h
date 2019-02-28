#ifndef SURGE_PID_SERVER_H
#define SURGE_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/surgePIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string>

class surgePIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::surgePIDAction> as_; // NodeHandle instance must be created before this line. 
                                                                         // Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::surgePIDFeedback feedback_;
    motion_layer::surgePIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor surge;

public:

    surgePIDAction(std::string);
    ~surgePIDAction();
    void goalCB();
    void preemptCB();
    void callback(const std_msgs::Float32ConstPtr&);

};
#endif 
