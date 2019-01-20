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

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/upwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>

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
    bool find (std::string, double);
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

    boost::thread* spin_thread;

    moveStraight move_straight;
    moveSideward move_sideward;
    moveDownward move_downward;
    depthStabilise depth_stabilise;

    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;

    motion_layer::sidewardPIDGoal sidewardPIDGoal;
    motion_layer::forwardPIDGoal forwardPIDGoal;
    motion_layer::upwardPIDGoal upwardPIDGoal;
    motion_layer::anglePIDGoal anglePIDGoal;

};
