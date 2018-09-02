#ifndef SINGLE_BUOY_H
#define SINGLE_BUOY_H

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/thread.hpp>
#include <string>

#include <move_forward_server.h>
#include <move_sideward_server.h>
#include <straight_server.h>

class singleBuoy {
public:
    singleBuoy();
    ~singleBuoy();
    void setActive(bool);
    void forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    
    moveForward move_forward_;
    moveSideward move_sideward_;
    moveStraight move_straight_;
    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    double forward_distance_;
    double depth_;
};
#endif // SINGLE_BUOY_H
