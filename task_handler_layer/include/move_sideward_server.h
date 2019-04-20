#ifndef MOVE_SIDEWARD_SERVER_H
#define MOVE_SIDEWARD_SERVER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/yawPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float32.h>
#include <boost/thread.hpp>
#include <string>
#include <mutex>

class moveSideward {

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;    
    motion_layer::yawPIDGoal yaw_PID_goal;

    boost::thread* spin_thread;
    std::mutex mtx;

public:

    moveSideward();
    ~moveSideward();

    void activate (int, std::string);
    void deActivate ();
    void setThrust(int);
    void spinThread();
};
#endif // MOVE_SIDEWARD_SERVER_H
