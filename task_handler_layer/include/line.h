#ifndef LINE_H
#define LINE_H

#include <ros/ros.h>

#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/Float32.h>

#include <boost/thread.hpp>
#include <string>

#include <task_handler.h>

class lineTask {
public:
    lineTask ();
    ~lineTask ();
    void setActive (bool);
    void spinThread ();
    void angleCB (const std_msgs::Float32ConstPtr&);

private:
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;

    motion_layer::sidewardPIDGoal sideward_PID_goal;
    motion_layer::upwardPIDGoal upward_PID_goal;
    motion_layer::anglePIDGoal angle_PID_goal;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    boost::thread* spin_thread;

    taskHandler th;

    double angle_ = 0;
    bool angleReceived = false;
};
#endif // LINE_H
