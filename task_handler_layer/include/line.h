#ifndef LINE_H
#define LINE_H

#include <ros/ros.h>

#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>
#include <motion_layer/surgePIDAction.h>

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
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;
    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;

    motion_layer::swayPIDGoal sway_PID_goal;
    motion_layer::surgePIDGoal surge_PID_goal;
    motion_layer::yawPIDGoal yaw_PID_goal;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    taskHandler th;

    std::mutex mtx;
};
#endif // LINE_H
