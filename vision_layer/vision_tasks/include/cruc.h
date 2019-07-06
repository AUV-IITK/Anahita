#ifndef CRUC
#define CRUC

#include "base_class.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

class Cruc : public Base_class
{
public:
    Cruc();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
        void InitTracker (cv::Mat& src_img, cv::Mat& thres_img);
        void updateTracker (cv::Mat& src_img, int& count1);
        cv::Point2f threshROI (const cv::Rect2d& bounding_rect, const cv::Mat& img, int padding);
        void updateCoordinates (cv::Point points);

private:
    image_transport::Publisher front_roi_pub;
    cv::Mat marked_img;
    bool initTracker = false;
    int countn=0;
        
    cv::Rect2d bbox1;
    cv::Rect2d bbox;
    cv::Ptr<cv::Tracker> tracker1;
    cv::Point TL;
    bool TL_init = false;

    double MAJOR;
    double MINOR;
    ros::ServiceServer service;
};

#endif // CRUC
