#pragma once

#include <base_class.h>
#include <master_layer/PingerFrontTarget.h>
#include <master_layer/PingerBottomTarget.h>

class Pinger : public Base_class {
public:
    Pinger ();
    void loadParams () override;
    void spinThreadFront () override;
    void spinThreadBottom () override;
    cv::Point getContourCenter (const std::vector<cv::Point>& contour);
    bool bottomTarget (master_layer::PingerBottomTarget::Request &req,
                       master_layer::PingerBottomTarget::Response &resp);
    bool frontTarget (master_layer::PingerFrontTarget::Request &req,
                      master_layer::PingerFrontTarget::Response &resp);

private:
    bool bottom_visible = false;
    bool front_visible = false;
    double bottom_size_thres = 300;
    double front_size_thres = 100;
    image_transport::Publisher front_roi_pub;
    ros::ServiceServer front_service;
    ros::ServiceServer bottom_service;
};