#ifndef PITCH_PID_SERVER_H
#define PITCH_PID_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <motion_layer/pitchPIDAction.h>
#include <errorToPWM.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

class pitchPIDAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<motion_layer::pitchPIDAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    motion_layer::pitchPIDFeedback feedback_;
    motion_layer::pitchPIDResult result_;
    double goal_;
    ros::Subscriber sub_;
    ErrorDescriptor pitch;

    boost::thread* spin_thread;

public:

    pitchPIDAction(std::string);
    ~pitchPIDAction();
    void goalCB();
    void preemptCB();
    void callBack(const std_msgs::Float32ConstPtr&);
};
#endif // PITCH_PID_SERVER_H
