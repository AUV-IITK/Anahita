#pragma once
#include "base_class.h"
#include <vision_tasks/ContourCenter.h>
#include <master_layer/TargetNormal.h>
#include <cmath>

class TestGate : public Base_class
{
public:
    TestGate();
    void loadParams() override;
    void spinThreadFront() override;
    void rectCB (const sensor_msgs::Image::ConstPtr &msg);
    bool getNormal (master_layer::TargetNormal::Request &req,
                    master_layer::TargetNormal::Response &resp);
private:
    image_transport::Publisher front_roi_pub;
    image_transport::Subscriber image_rect_sub;
    ros::ServiceClient contour_center_client;
    ros::ServiceServer normal_server;
    cv::Mat rect_image;
    cv::Mat rect_thresholded;
};
