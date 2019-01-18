#ifndef LINE_H
#define LINE_H

#include <ros/ros.h>

#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/forwardPIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/Float32.h>

#include <boost/thread.hpp>
#include <string>
#include <mutex>

#include <task_handler.h>

class lineTask {
public:
    lineTask ();
    ~lineTask ();
    bool setActive (bool);

private:
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;

    motion_layer::sidewardPIDGoal sideward_PID_goal;
    motion_layer::forwardPIDGoal forward_PID_goal;
    motion_layer::anglePIDGoal angle_PID_goal;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    taskHandler th;

    std::mutex mtx;
};
#endif // LINE_H
