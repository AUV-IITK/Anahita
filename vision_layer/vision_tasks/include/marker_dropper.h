#pragma once

#include "base_class.h"

class MarkerDropper : public Base_class {
public:
    MarkerDropper ();
    virtual void loadParams () override;
    virtual void spinThreadBottom () override;
    void preProcess (cv::Mat& temp_src);
    cv::Point findCenter ();

private:
    cv::Scalar bgr_min;
    cv::Scalar bgr_max;
};
