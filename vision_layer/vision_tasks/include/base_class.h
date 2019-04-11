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
#include <geometry_msgs/Point32.h>
#include <stdlib.h>
#include <string>
#include <boost/thread.hpp> 

#include <filter.h>
#include <contour.h>
#include <morph.h>
#include <threshold.h>
#include <geometry.h>

class Base_class{

    protected:
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

        bool close_task= false;

        image_transport::Subscriber front_image_sub;
        image_transport::Subscriber bottom_image_sub;

	image_transport::Publisher bottom_thresholded_pub;
	image_transport::Publisher bottom_marked_pub;
	
	image_transport::Publisher front_thresholded_pub;
	image_transport::Publisher front_marked_pub;

        ros::Publisher front_x_coordinates_pub;
        ros::Publisher front_y_coordinates_pub;
        ros::Publisher front_z_coordinates_pub;
        ros::Publisher bottom_x_coordinates_pub;
        ros::Publisher bottom_y_coordinates_pub;
        ros::Publisher bottom_z_coordinates_pub;

        ros::Publisher front_coordinates_pub;
        ros::Publisher bottom_coordinates_pub;
   	ros::Publisher detection_pub;
    
      public:
        Base_class();
        ros::NodeHandle nh;
        image_transport::ImageTransport it();

        std_msgs::Float32 front_x_coordinate;
	std_msgs::Float32 front_y_coordinate;
        std_msgs::Float32 front_z_coordinate;

        std_msgs::Float32 bottom_x_coordinate;
	std_msgs::Float32 bottom_y_coordinate;
        std_msgs::Float32 bottom_z_coordinate;


        cv::Mat image_front;
        cv::Mat image_bottom;
	cv::Mat image_front_marked;
        cv::Mat image_bottom_marked;
        cv::Mat image_front_thresholded;
        cv::Mat image_bottom_thresholded;

        void spinThreadFront();
        void spinThreadBottom();
        void bottomTaskHandling(bool status);
        void frontTaskHandling(bool status);
        void loadParam();
        void init();

        boost::thread* spin_thread_bottom; 
        boost::thread* spin_thread_front; 
};
#endif 

