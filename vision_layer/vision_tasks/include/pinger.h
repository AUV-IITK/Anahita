#pragma once

#include <base_class.h>

class Pinger : public Base_class {
public:
    Pinger ();
    void loadParams () override;
    void spinThreadFront () override;
    void spinThreadBottom () override;
    cv::Point getContourCenter (const std::vector<cv::Point>& contour);

private:
    bool bottom_visible = false;
    bool front_visible = false;
    double bottom_size_thres = 300;
    double front_size_thres = 100;
    image_transport::Publisher front_roi_pub;
};