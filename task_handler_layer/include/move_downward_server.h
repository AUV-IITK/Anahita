#ifndef MOVE_DOWNWARD_SERVER_H
#define MOVE_DOWNWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <boost/thread.hpp>
#include <string>
#include <mutex>

class moveDownward {

protected:

    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;    
    motion_layer::anglePIDGoal angle_PID_goal;

    boost::thread* spin_thread;

    int pwm = 0;

public:

    moveDownward(int);
    ~moveDownward();

    void setActive(bool, std::string);
    void setThrust(int);
    void spinThread();
};
#endif // MOVE_DOWNWARD_SERVER_H
