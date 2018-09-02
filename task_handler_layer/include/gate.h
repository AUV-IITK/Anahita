#ifndef GATE_H
#define GATE_H

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/thread.hpp>
#include <string>

#include <straight_server.h>
#include <move_forward_server.h>

class gateTask
{
private:
    moveStraight move_straight_;
    moveForward move_forward_;
    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    double distance;
    boost::thread* spin_thread;

public:
    gateTask();
    ~gateTask();

    void setActive(bool);
    void spinThread();
    void distanceCB(const geometry_msgs::PointStamped::ConstPtr &_msg); 
};

#endif // GATE_H