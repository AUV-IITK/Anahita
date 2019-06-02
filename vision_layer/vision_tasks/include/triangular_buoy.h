#pragma once

#include <base_class.h>
#include <geometry_msgs/Point.h>
#include <master_layer/GetMaxDepth.h>

class TriangularBuoy : public Base_class {
public:
    TriangularBuoy();
    void loadParams () override;
    void spinThreadFront () override;
    void preProcess (cv::Mat& temp_src);
    cv::Point findCenterAndSpeed ();
    void depthCallback (const geometry_msgs::Point msg);
    bool depthRequest (master_layer::GetMaxDepth::Request& req,
                       master_layer::GetMaxDepth::Response& res);
private:
    image_transport::Publisher roi_pub;
    ros::ServiceServer service;
    ros::Subscriber depth_sub;

    cv::Scalar bgr_min;
    cv::Scalar bgr_max;

    float max_depth = 0;
    float rotation_speed = 0;
};
