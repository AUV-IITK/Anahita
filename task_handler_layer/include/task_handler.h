#pragma once

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

#include <mutex>

using namespace std;

class taskHandler {
public:
    taskHandler (double _timeout);
    ~taskHandler ();
    bool isAchieved (double _target, double _band, std::string _topic);
    bool isDetected (std::string _task, double _timeout);
    void callBack (const std_msgs::Float32Ptr &_msg);
    void visionCB (const std_msgs::BoolPtr& _msg);
    void setTimeout(double _time);
    void setVisionTimeout(double _time);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Subscriber vision_sub_;

    std::map<std::string, std::string> topic_map_;
    std::map<std::string, bool> task_map_;

    double data_;
    double time_out_;
    double vision_time_out_;

    bool dataReceived = false;

    std::mutex data_mutex;
    std::mutex vision_mutex;
    std::mutex mtx;

    bool is_subscribed_= false;
};
