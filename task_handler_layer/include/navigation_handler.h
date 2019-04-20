#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_downward_server.h>
#include <depth_stabilise.h>

#include <motion_layer/surgePIDAction.h>
#include <motion_layer/heavePIDAction.h>
#include <motion_layer/swayPIDAction.h>
#include <motion_layer/yawPIDAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <task_handler.h>

#include <mutex>

using namespace std;

class navigationHandler {
public:
    navigationHandler ();
    ~navigationHandler ();

    void stabilise (std::string);
    void manouver ();
    bool find (std::string, double, int);
    bool isStable ();
    void findNextTask ();
    void analyze ();
    bool scan (std::string);
    bool dive (std::string);

private:
    ros::NodeHandle nh;

    taskHandler th;

    bool stop_manouver = false;
    std::mutex mtx;
    int direction = 1;

    boost::thread* spin_thread;

    moveStraight move_straight;
    moveSideward move_sideward;
    moveDownward move_downward;
    depthStabilise depth_stabilise;

    actionlib::SimpleActionClient<motion_layer::surgePIDAction> surgePIDClient;
    actionlib::SimpleActionClient<motion_layer::heavePIDAction> heavePIDClient;
    actionlib::SimpleActionClient<motion_layer::swayPIDAction> swayPIDClient;
    actionlib::SimpleActionClient<motion_layer::yawPIDAction> yawPIDClient;

    motion_layer::swayPIDGoal swayPIDGoal;
    motion_layer::surgePIDGoal surgePIDGoal;
    motion_layer::heavePIDGoal heavePIDGoal;
    motion_layer::yawPIDGoal yawPIDGoal;

};
