#ifndef BUOY_TASK_H
#define BUOY_TASK_H

#include "base_class.h"
#include <std_msgs/Int32MultiArray.h>

class Buoy : public Base_class
{
public:
	Buoy();
    void loadParams () override;
	void spinThreadFront() override;
    cv::Mat preprocess (cv::Mat);
    void extractFeatures (const cv::Mat&);

private:
    image_transport::Publisher front_roi_pub;
    ros::Publisher features_pub;
    std_msgs::Int32MultiArray feature_msg;
};
#endif // BUOY_TASK_H
