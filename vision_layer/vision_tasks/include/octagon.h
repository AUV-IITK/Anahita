#ifndef OCTAGON_TASK_H
#define OCTAGON_TASK_H

#include "base_class.h" 

#include <vision_tasks/octagonFrontRangeConfig.h>
#include <vision_tasks/octagonBottomRangeConfig.h>

class Octagon : public Base_class
{
protected:
	
    image_transport::Publisher blue_filtered_pub;
	image_transport::Subscriber image_raw_sub;

	std::string camera_frame_;

    double front_clahe_clip_ = 4.0;
    int front_clahe_grid_size_ = 8;
    int front_clahe_bilateral_iter_ = 8;
    int front_balanced_bilateral_iter_ = 4;
    double front_denoise_h_ = 10.0;
    int front_canny_threshold_low_ = 0;
    int front_canny_threshold_high_ = 1000;
    int front_canny_kernel_size_ = 3;
    int front_hough_threshold_ = 0;
    int front_hough_minline_ = 0;
    int front_hough_maxgap_ = 0;
    double front_hough_angle_tolerance_ = 0.0;
    double front_gate_distance_tolerance_ = 50.0;
    double front_gate_angle_tolerance_ = 0.0;

    double bottom_clahe_clip_ = 4.0;
    int bottom_clahe_grid_size_ = 8;
    int bottom_clahe_bilateral_iter_ = 8;
    int bottom_balanced_bilateral_iter_ = 4;
    double bottom_denoise_h_ = 10.0;
    
    bool task_done = false;

	void frontCallback(vision_tasks::octagonFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::octagonBottomRangeConfig &config, double level);
	void imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg);
    void imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg);

public:
    Octagon();
	void spinThreadBottom();
    void spinThreadFront();
};
#endif // OCTAGON_TASK_H

