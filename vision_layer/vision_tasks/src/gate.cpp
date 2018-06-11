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
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>

#include <vision_tasks/gateRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>

int low_h, low_s, low_v, high_h, high_s, high_v;
image_transport::Publisher thresholded_HSV_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;
std::string camera_frame = "auv-iitk";  

void callback(vision_tasks::gateRangeConfig &config, double level){
    ROS_INFO("Reconfigure Request: (%d %d %d) - (%d %d %d): ", config.low_h, config.low_s, config.low_v, config.high_h, config.high_s, config.high_v);
	low_h = config.low_h;
	low_s = config.low_s;
	low_v = config.low_v;
	high_h = config.high_h;
	high_s = config.high_s;
	high_v = config.high_v;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_img_ptr->image;
        cv::Mat image_marked = cv_img_ptr->image;

        if(!image.empty()){
            cv::Mat image_hsv;
            cv::Mat image_thresholded;
            cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV);
            
            ROS_INFO("Thresholding Values: (%d %d %d) - (%d %d %d): ", low_h, low_s, low_v, high_h, high_s, high_v);

            if(!(high_h<=low_h || high_s<=low_s || high_v<=low_v)) {
		
				image_thresholded = vision_commons::Threshold::threshold(image_hsv, low_h, low_s, low_v, high_h, high_s, high_v);

				image_thresholded = vision_commons::Morph::open(image_thresholded, 3, 1, 1, 2);
				image_thresholded = vision_commons::Morph::close(image_thresholded, 3, 1, 1, 2);
				
				cv_bridge::CvImage thresholded_ptr;
				thresholded_ptr.header = msg->header;
				thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
				thresholded_ptr.image = image_thresholded;
				thresholded_HSV_pub.publish(thresholded_ptr.toImageMsg());

   				std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 2);
                ROS_INFO("contours size = %d", contours.size());
                
                if(contours.size()!=0){
   					cv::Rect bounding_rectangle = cv::boundingRect(cv::Mat(contours[0]));
                    cv::rectangle(image_marked, bounding_rectangle, cv::Scalar(255,0,255), 1, 8, 0);

                    cv_bridge::CvImage marked_ptr;
					marked_ptr.header = msg->header;
					marked_ptr.encoding = sensor_msgs::image_encodings::RGB8;
					marked_ptr.image = image_marked;
					marked_pub.publish(marked_ptr.toImageMsg());

                }
                


            }

        }   
    }
    catch(cv_bridge::Exception &e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch(cv::Exception &e){
		ROS_ERROR("cv exception: %s", e.what());
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "gate_task");
	ros::NodeHandle nh;
	dynamic_reconfigure::Server<vision_tasks::gateRangeConfig> server;
	dynamic_reconfigure::Server<vision_tasks::gateRangeConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	image_transport::ImageTransport it(nh);
	thresholded_HSV_pub = it.advertise("/thresholded", 1);
	marked_pub = it.advertise("/marked",1);
	//coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/threshold/gate_center_coordinates", 1000);
	image_transport::Subscriber image_raw_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
	ros::spin();
	return 0;
}
