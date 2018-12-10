#ifndef FORWARD_PID_SERVER_H
#define FORWARD_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/forwardPIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <string>

class forwardPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::forwardPIDAction> as_; // NodeHandle instance must be created before this line. 
                                                                         // Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::forwardPIDFeedback feedback_;
    motion_layer::forwardPIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ErrorDescriptor x_coord;

public:

    forwardPIDAction(std::string);
    ~forwardPIDAction();
    void goalCB();
    void preemptCB();
    void visionCB(const std_msgs::Float32ConstPtr&);

};
#endif 
