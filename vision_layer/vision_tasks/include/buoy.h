#ifndef BUOY_TASK_H
#define BUOY_TASK_H

#include "base_class.h" 
#include <vision_tasks/buoyRangeConfig.h>

class Buoy : public Base_class
{
protected:

    ros::NodeHandle nh;
    image_transport::Publisher blue_filtered_pub;
	image_transport::Subscriber image_raw_sub;

	std::string camera_frame_;
    double clahe_clip_;
	int clahe_grid_size_;
	int clahe_bilateral_iter_;
	int balanced_bilateral_iter_;
	double denoise_h_;
	int data_low_h[3] = {0, 12, 0};
	int data_high_h[3] = {17, 40, 56};
	int data_low_s[3] = {206, 183, 0};
	int data_high_s[3] = {255, 255, 255};
	int data_low_v[3] = {30, 3, 2};
	int data_high_v[3] = {255, 255, 82};
	void callback(vision_tasks::buoyRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
	Buoy();
	void switchColor(int);
	void spinThreadFront();
	void spinThread();
	int current_color;
};
#endif // BUOY_TASK_H
