#ifndef SINGLE_BUOY_H
#define SINGLE_BUOY_H

#include <ros/ros.h>

#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/anglePIDAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <boost/thread.hpp>
#include <string>

class singleBuoy {
public:
    singleBuoy();
    ~singleBuoy();
    void setActive(bool);
    void forwardCB(const geometry_msgs::PointStamped::ConstPtr &_msg);
    void sidewardCB(const geometry_msgs::PointStamped::ConstPtr &_msg);
    void angleCB(const std_msgs::Float64Ptr &_msg);

private:
    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient;
    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient;
    actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient;
    
    ros::Subscriber forward_sub_;
    ros::Subscriber sideward_sub_;
    ros::Subscriber angle_sub_;

    ros::NodeHandle nh_;
    double forward_distance_;
    double sideward_distance_;
    double angle_;
};
#endif // SINGLE_BUOY_H
