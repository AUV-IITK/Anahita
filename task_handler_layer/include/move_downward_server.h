#ifndef MOVE_DOWNWARD_SERVER_H
#define MOVE_DOWNWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <string>

class moveDownward {

protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;

public:

    moveDownward(int);
    ~moveDownward();

    boost::thread* spin_thread;
    void setActive(bool);
    void setThrust(int);
    void spinThread();
};
#endif // MOVE_DOWNWARD_SERVER_H
