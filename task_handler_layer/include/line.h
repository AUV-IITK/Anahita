#ifndef LINE_H
#define LINE_H

#include <ros/ros.h>

#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/thread.hpp>
#include <string>

#include <move_forward_server.h>
#include <straight_server.h>

class lineTask {
public:
    lineTask();
    ~lineTask();
    void setActive(bool);
    void spinThread();

private:
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    moveForward move_forward_;
    moveStraight move_straight_;
    ros::NodeHandle nh_;
    double angle;
    boost::thread* spin_thread;
    motion_layer::sidewardPIDGoal sideward_PID_goal;
    motion_layer::upwardPIDGoal upward_PID_goal;
    motion_layer::anglePIDGoal angle_PID_goal;
};
#endif // LINE_H
