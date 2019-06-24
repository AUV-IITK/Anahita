#pragma once

#include "base_class.h"
#include <std_msgs/Int32MultiArray.h>

class StartGate : public Base_class
{
public:
    StartGate ();
    void loadParams() override;
    void spinThreadFront() override;
    cv::Point findGateCenter (cv::Mat& thres_img);
    void extractFeatures (const cv::Mat& thres_img);

private:
    image_transport::Publisher front_roi_pub;
    ros::Publisher features_pub;
    cv::Mat marked_img;
    std_msgs::Int32MultiArray feature_msg;
};
