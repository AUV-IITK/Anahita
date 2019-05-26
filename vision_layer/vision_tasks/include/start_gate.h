#pragma once

#include "base_class.h"

class StartGate : public Base_class
{
public:
    StartGate ();
    void loadParams() override;
    void spinThreadFront() override;
    cv::Point findGateCenter (cv::Mat& thres_img);

private:
    image_transport::Publisher front_roi_pub;
    cv::Mat marked_img;
};
