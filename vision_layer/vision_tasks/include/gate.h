#ifndef GATE_TASK_H
#define GATE_TASK_H

#include "base_class.h"

#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_tasks/gateBottomRangeConfig.h>

class Gate : public Base_class
{
protected:
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

	image_transport::Publisher blue_filtered_pub_front;
	image_transport::Publisher canny_pub_front;
	image_transport::Publisher lines_pub_front;
	
    image_transport::Publisher blue_filtered_pub_bottom;
    ros::Publisher task_done_pub;

    std::string camera_frame_;
	void frontCallback(vision_tasks::gateFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::gateBottomRangeConfig &config, double level);    
	void imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg);
    void imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle);

public:
    Gate();
	void bottomTaskHandling(bool status);
    void frontTaskHandling(bool status);
    void spinThreadFront();
    void spinThreadBottom();
};
#endif // GATE_TASK_H
