#ifndef BASE_CLASS_H
#define BASE_CLASS_H

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <boost/thread.hpp> 

#include <vision_commons/filter.h>
#include <vision_commons/contour.h>
#include <vision_commons/morph.h>
#include <vision_commons/threshold.h>
#include <vision_commons/geometry.h>

class Base_class{

    protected:
        double front_clahe_clip_;
        int front_clahe_grid_size_ ;
        int front_clahe_bilateral_iter_;
        int front_balanced_bilateral_iter_;
        double front_denoise_h_;
        int front_low_h_;
        int front_high_h_;
        int front_low_s_;
        int front_high_s_;
        int front_low_v_;
        int front_high_v_;
        int front_opening_mat_point_;	
	int front_opening_iter_ ;
        int front_closing_mat_point_;
        int front_closing_iter_;
/*        int front_canny_threshold_low_;
        int front_canny_threshold_high_;
        int front_canny_kernel_size_;
        int front_hough_threshold_;
        int front_hough_minline_;
        int front_hough_maxgap_ ;
        double front_hough_angle_tolerance_ ;
        double front_gate_distance_tolerance_ ;
        double front_gate_angle_tolerance_ ;*/

        double bottom_clahe_clip_ ;
        int bottom_clahe_grid_size_ ;
        int bottom_clahe_bilateral_iter_;
        int bottom_balanced_bilateral_iter_;
        double bottom_denoise_h_ ;
        int bottom_low_h_ ;
        int bottom_low_s_;
        int bottom_low_v_;
        int bottom_high_h_;
        int bottom_high_s_;
        int bottom_high_v_;
        int bottom_closing_mat_point_;
        int bottom_closing_iter_;
        int bottom_opening_iter_;    
        int bottom_opening_mat_point_;

	bool task_done = false;

        //image_transport::Publisher thresholded_pub;
    	image_transport::Publisher marked_pub;
        image_transport::Subscriber image_raw_sub;

        image_transport::Subscriber front_image_sub;
        image_transport::Subscriber bottom_image_sub;

      //image_transport::Publisher blue_filtered_pub;
    	image_transport::Publisher thresholded_pub;
	image_transport::Publisher marked_pub;
	    //image_transport::Publisher blue_filtered_pub_front;
	image_transport::Publisher thresholded_pub_front; 
//	    image_transport::Publisher canny_pub_front;
//	    image_transport::Publisher lines_pub_front;
	image_transport::Publisher marked_pub_front;
//      image_transport::Publisher blue_filtered_pub_bottom;
//        image_transport::Publisher thresholded_pub_bottom;
//      image_transport::Publisher marked_pub_bottom;
        image_transport::Publisher bottom_blue_filtered_pub;
	image_transport::Publisher bottom_thresholded_pub;
	image_transport::Publisher bottom_marked_pub;
	ros::Publisher bottom_coordinates_pub;
	
	image_transport::Publisher front_blue_filtered_pub;
	image_transport::Publisher front_thresholded_pub;
	image_transport::Publisher front_marked_pub;
	ros::Publisher front_coordinates_pub;

	image_transport::Subscriber front_image_raw_sub;
	image_transport::Subscriber bottom_image_raw_sub;

        ros::Publisher x_coordinates_pub;
	ros::Publisher y_coordinates_pub;
	ros::Publisher z_coordinates_pub;
        ros::Publisher coordinates_pub_bottom;
        ros::Publisher task_done_pub
   	ros::Publisher detection_pub;
	ros::Publisher coordinates_pub;
        ros::Publisher ang_pub;

        std::string camera_frame_;

//        void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
        void imageFrontCallback(const sensor_msgs::Image::ConstPtr &msg);
        void imageBottomCallback(const sensor_msgs::Image::ConstPtr &msg);
        void frontCallback(vision_tasks::gateFrontRangeConfig &config, double level);
	void bottomCallback(vision_tasks::gateBottomRangeConfig &config, double level);
//        cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle);
    
    public:
        Base_class();
        ros::NodeHandle nh;
        image_transport::ImageTransport it();
        std_msgs::Float32 x_coordinate;
	std_msgs::Float32 y_coordinate;
        std_msgs::Float32 z_coordinate;

        cv::Mat image_;
        cv::Mat image_front;
        cv::Mat image_bottom;
	cv::Mat image_front_marked;
        cv::Mat image_bottom_marked;
//change image_marked accordingly

        void bottomTaskHandling(bool status);
        void frontTaskHandling(bool status);

//        boost::thread* spin_thread;
        boost::thread* spin_thread_bottom; 
        boost::thread* spin_thread_front; 
};
#endif 

