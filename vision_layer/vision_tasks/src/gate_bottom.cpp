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
#include <std_msgs/Bool.h>
#include <vision_tasks/gateBottomRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
int low_b = 10;
int low_g = 0;
int low_r = 0;
int high_b = 90;
int high_g= 255;
int high_r = 255;
int closing_mat_point = 1;
int closing_iter = 1;

image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;

ros::Publisher task_done_pub;
std::string camera_frame = "auv-iitk";

void callback(vision_tasks::gateBottomRangeConfig &config, double level){
  clahe_clip = config.clahe_clip;
  clahe_grid_size = config.clahe_grid_size;
  clahe_bilateral_iter = config.clahe_bilateral_iter;
  balanced_bilateral_iter = config.balanced_bilateral_iter;
  denoise_h = config.denoise_h;
  low_b = config.low_b;
  low_g = config.low_g;
  low_r = config.low_r;
  high_b = config.high_b;
  high_g = config.high_g;
  high_r = config.high_r;
  closing_mat_point = config.closing_mat_point;
  closing_iter = config.closing_iter;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_img_ptr->image;
    cv::Mat image_marked = cv_img_ptr->image;
    if(!image.empty()){
      cv::Mat blue_filtered = vision_commons::Filter::blue_filter(image, clahe_clip, clahe_grid_size, clahe_bilateral_iter, balanced_bilateral_iter, denoise_h);
      blue_filtered_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
      cv::Mat image_thresholded;
      if(!(high_b<=low_b || high_g<=low_g || high_r<=low_r)) {
        image_thresholded = vision_commons::Threshold::threshold(image, low_r, low_g, low_b, high_r, high_g, high_b);
        image_thresholded = vision_commons::Morph::close(image_thresholded, 2*closing_mat_point+1, closing_mat_point, closing_mat_point, closing_iter);
        cv_bridge::CvImage thresholded_ptr;
        thresholded_ptr.header = msg->header;
        thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
        thresholded_ptr.image = image_thresholded;
        thresholded_pub.publish(thresholded_ptr.toImageMsg());
        std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 1);
        if(contours.size()!=0){
          cv::RotatedRect bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
          geometry_msgs::PointStamped pipe_point_message;
          pipe_point_message.header.stamp = ros::Time();
          pipe_point_message.header.frame_id = camera_frame.c_str();
          pipe_point_message.point.x = (image.size().height)/2 - bounding_rectangle.center.y;
          pipe_point_message.point.y = bounding_rectangle.center.x - (image.size().width)/2;
          pipe_point_message.point.z = 0.0;
          coordinates_pub.publish(pipe_point_message);
          ROS_INFO("Pipe Center (x, y) = (%.2f, %.2f)", pipe_point_message.point.x, pipe_point_message.point.y);
          std_msgs::Bool task_done_message;
          task_done_message.data = pipe_point_message.point.x < 0;
          task_done_pub.publish(task_done_message);
          ROS_INFO("Task done (bool) = %s", task_done_message.data ? "true" : "false");
          cv::Point2f vertices2f[4];
          bounding_rectangle.points(vertices2f);
          cv::Point vertices[4];
          for(int i = 0; i < 4; ++i) {
            vertices[i] = vertices2f[i];
          }
          cv::fillConvexPoly(image, vertices, 4, cv::Scalar(0,155,155));
          cv::circle(image_marked, bounding_rectangle.center, 1, cv::Scalar(255,100,100), 8, 0);
          cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(255,100, 50), 8, 0);
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
  ros::init(argc, argv, "gate_task_bottom");
  ros::NodeHandle nh;
  dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig> server;
  dynamic_reconfigure::Server<vision_tasks::gateBottomRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  image_transport::ImageTransport it(nh);
  blue_filtered_pub = it.advertise("/gate_task/bottom/blue_filtered", 1);
  thresholded_pub = it.advertise("/gate_task/bottom/thresholded", 1);
  marked_pub = it.advertise("/gate_task/bottom/marked", 1);
  coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/bottom/pipe_coordinates", 1000);
  task_done_pub = nh.advertise<std_msgs::Bool>("/gate_task/done", 1000);
  image_transport::Subscriber bottom_image_sub = it.subscribe("/bottom_camera/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}
