#ifndef MARKER_H
#define MARKER_H

#include "base_class.h"
#include <std_msgs/Int32MultiArray.h>

class Marker : public Base_class
{
public:
    Marker();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    void extractFeatures (const cv::Mat& thres_img);

private:
    image_transport::Publisher front_roi_pub;
    ros::Publisher features_pub;
    std_msgs::Int32MultiArray feature_msg;
};

#endif // MARKER_H
