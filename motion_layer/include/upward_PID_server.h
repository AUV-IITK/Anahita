#ifndef UPWARD_PID_SERVER_H
#define UPWARD_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/upwardPIDAction.h>
#include <errorToPWM.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

class upwardPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::upwardPIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::upwardPIDFeedback feedback_;
    motion_layer::upwardPIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor z_coord; // z_coord for forward direction

public:

    upwardPIDAction(std::string);
    ~upwardPIDAction();
    void goalCB();
    void preemptCB();
    void depthCB(const std_msgs::Float32ConstPtr&);
};
#endif // UPWARD_PID_SERVER_H
