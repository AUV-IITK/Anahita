#pragma once

#include <base_class.h>
#include <geometry_msgs/Point.h>
#include <master_layer/GetMaxDepth.h>
#include <master_layer/GetBlackoutTime.h>

class TriangularBuoy : public Base_class {
public:
    TriangularBuoy();
    void loadParams () override;
    void spinThreadFront () override;
    void preProcess (cv::Mat& temp_src);
    cv::Point findCenter ();
    void depthCallback (const geometry_msgs::Point msg);
    bool depthRequest (master_layer::GetMaxDepth::Request& req,
                       master_layer::GetMaxDepth::Response& res);
    bool blackoutCB (master_layer::GetBlackoutTime::Request& req,
                     master_layer::GetBlackoutTime::Response& res);
    void rectCB (const sensor_msgs::Image::ConstPtr &msg);

private:
    image_transport::Publisher roi_pub;
    image_transport::Subscriber image_rect_sub;

    ros::ServiceServer depth_service;
    ros::ServiceServer blackout_service;
    ros::Subscriber depth_sub;

    cv::Scalar bgr_min;
    cv::Scalar bgr_max;

    float max_depth = 0;
    float blackout_time = 0;

    cv::Mat rect_image;
    cv::Mat rect_thresholded;

    bool init = false;
    float init_time = 0;
    bool contour_init = false;
    bool blackout_response_ready = false;
};
