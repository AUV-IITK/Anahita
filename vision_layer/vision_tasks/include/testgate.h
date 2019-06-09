#pragma once
#include "base_class.h"
#include <vision_tasks/ContourCenter.h>

class StartGate : public Base_class
{
public:
    StartGate();
    void loadParams() override;
    void spinThreadFront() override;
    void rectCB (const sensor_msgs::Image::ConstPtr &msg);

private:
    image_transport::Publisher front_roi_pub;
    image_transport::Subscriber image_rect_sub;
    ros::ServiceClient contour_center_client;
    cv::Mat rect_image;
};
