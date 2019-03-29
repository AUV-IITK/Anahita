#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

#include "base_class.h"

#include <vision_tasks/torpedoRangeConfig.h>

class Torpedo : public Base_class
{
protected:

    image_transport::Publisher blue_filtered_pub;
	
	image_transport::Subscriber image_raw_sub;
	std::string camera_frame_;

    double clahe_clip_;
	int clahe_grid_size_;
	int clahe_bilateral_iter_;
	int balanced_bilateral_iter_;
	double denoise_h_;
	
	int data_low_h[2] = {43, 0};
	int data_high_h[2] = {74, 13};
	int data_low_s[2] = {41, 220};
	int data_high_s[2] = {255, 255};
	int data_low_v[2] = {0, 87};
	int data_high_v[2] = {255, 181};

	void callback(vision_tasks::torpedoRangeConfig &config, double level);
	void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    Torpedo();
	void switchColor(int);
	void spinThreadFront();
	int current_color;
};
#endif // TORPEDO_TASK_H

