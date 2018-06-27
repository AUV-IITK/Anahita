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
#include <vision_tasks/gateRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>

//cv::Mat horizontalkern = (cv::Mat_<float>(3,3) << 1,1,1,0,0,0,1,1,1)/(float)(9);
//cv::Mat verticalkern = (cv::Mat_<float>(3,3) << 1,0,1,1,0,1,1,0,1)/(float)(9);
cv::Mat kern = (cv::Mat_<float>(3,3) << 1, 0, 0, 1, 0, 0, 1, 1, 1)/(float)(9);

double front_clahe_clip = 4.0;
int front_clahe_grid_size = 8;
int front_clahe_bilateral_iter = 8;
int front_balanced_bilateral_iter = 4;
double front_denoise_h = 10.0;
int front_low_b = 10;
int front_low_g = 0;
int front_low_r = 0;
int front_high_b = 90;
int front_high_g= 255;
int front_high_r = 255;
int front_closing_mat_point = 1;
int front_closing_iter = 1;
int front_canny_threshold_low = 0;
int front_canny_threshold_high = 0;
int front_hough_threshold = 0;
int front_hough_minline = 0;
int front_hough_maxgap = 0;

double bottom_clahe_clip = 4.0;
int bottom_clahe_grid_size = 8;
int bottom_clahe_bilateral_iter = 8;
int bottom_balanced_bilateral_iter = 4;
double bottom_denoise_h = 10.0;
int bottom_low_b = 10;
int bottom_low_g = 0;
int bottom_low_r = 0;
int bottom_high_b = 90;
int bottom_high_g= 255;
int bottom_high_r = 255;
int bottom_closing_mat_point = 1;
int bottom_closing_iter = 1;

image_transport::Publisher blue_filtered_front_pub;
image_transport::Publisher thresholded_front_pub;
image_transport::Publisher convolved_front_pub;
image_transport::Publisher marked_front_pub;
image_transport::Publisher lines_front_pub;
ros::Publisher coordinates_front_pub;

image_transport::Publisher blue_filtered_bottom_pub;
image_transport::Publisher thresholded_bottom_pub;
image_transport::Publisher marked_bottom_pub;
ros::Publisher coordinates_bottom_pub;

ros::Publisher task_done_pub;
std::string camera_frame = "auv-iitk";

float angleWrtY(const cv::Point2f &v1, const cv::Point2i &v2)
{
  cv::Point2f v3;
  v3.x = v1.x - v2.x;
  v3.y = v1.y - v2.y;
  float angle = atan2(v3.y, v3.x);
  return angle*180/CV_PI;
}

cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle)
{
  if(v1.x > v2.x)
  {
    cv::Point2i v3 = v1 - v2;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v2;
    return finalVertex;
  }
  else
  {
    cv::Point2i v3 = v2 - v1;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v1;
    return finalVertex;
  }
}


void callback(vision_tasks::gateRangeConfig &config, double level){
  front_clahe_clip = config.front_clahe_clip;
  front_clahe_grid_size = config.front_clahe_grid_size;
  front_clahe_bilateral_iter = config.front_clahe_bilateral_iter;
  front_balanced_bilateral_iter = config.front_balanced_bilateral_iter;
  front_denoise_h = config.front_denoise_h;
  front_low_b = config.front_low_b;
  front_low_g = config.front_low_g;
  front_low_r = config.front_low_r;
  front_high_b = config.front_high_b;
  front_high_g = config.front_high_g;
  front_high_r = config.front_high_r;
  front_closing_mat_point = config.front_closing_mat_point;
  front_closing_iter = config.front_closing_iter;
  front_canny_threshold_low = config.front_canny_threshold_low;
  front_canny_threshold_high = config.front_canny_threshold_high;
  front_hough_threshold = config.front_hough_threshold;
  front_hough_minline = config.front_hough_minline;
  front_hough_maxgap = config.front_hough_maxgap;
  bottom_clahe_clip = config.bottom_clahe_clip;
  bottom_clahe_grid_size = config.bottom_clahe_grid_size;
  bottom_clahe_bilateral_iter = config.bottom_clahe_bilateral_iter;
  bottom_balanced_bilateral_iter = config.bottom_balanced_bilateral_iter;
  bottom_denoise_h = config.bottom_denoise_h;
  bottom_low_b = config.bottom_low_b;
  bottom_low_g = config.bottom_low_g;
  bottom_low_r = config.bottom_low_r;
  bottom_high_b = config.bottom_high_b;
  bottom_high_g = config.bottom_high_g;
  bottom_high_r = config.bottom_high_r;
  bottom_closing_mat_point = config.bottom_closing_mat_point;
  bottom_closing_iter = config.bottom_closing_iter;
}

void imageFrontCallback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_img_ptr->image;
    cv::Mat image_marked = cv_img_ptr->image;
    if(!image.empty()){
      cv::Mat blue_filtered = vision_commons::Filter::blue_filter(image, front_clahe_clip, front_clahe_grid_size, front_clahe_bilateral_iter, front_balanced_bilateral_iter, front_denoise_h);
      blue_filtered_front_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
      if(front_canny_threshold_high > front_canny_threshold_low) {
        //std::vector<cv::Mat> horizontal_planes(3);
        //std::vector<cv::Mat> vertical_planes(3);
        //cv::split(blue_filtered, horizontal_planes);
        //cv::split(blue_filtered, vertical_planes);
        //cv::Mat horizontal, vertical;
        //for(int j = 0; j<3; j++){
          //for(int i = 0; i<3; i++)
            //cv::filter2D(horizontal_planes[j], horizontal_planes[j], horizontal_planes[j].depth(), horizontalkern);
          //for(int i = 0; i<3; i++)
            //cv::filter2D(vertical_planes[j], vertical_planes[j], vertical_planes[j].depth(), verticalkern);
        //}
        //merge(horizontal_planes, horizontal);
        //merge(vertical_planes, vertical);
        std::vector<cv::Mat> planes(3);
        cv::split(blue_filtered, planes);
        for(int j = 0 ; j < 3 ; j++) {
          for(int i = 0 ; i < 3 ; i++) cv::filter2D(planes[j], planes[j], planes[j].depth(), kern);
        }
        cv::Mat convolved;
        merge(planes, convolved);
        //cv::addWeighted(horizontal, 0.5, vertical, 0.5, 0.0,  convolved);
        cv::Mat copy;
        convolved.copyTo(copy);
        copy.convertTo(convolved, -1, 5.0, 0.0);
        //convolved = vision_commons::Filter::clahe(convolved, 4.0, 11);
        convolved_front_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", convolved).toImageMsg());

        std::vector<cv::Vec4i> lines;
        cv::Mat image_thresholded = vision_commons::Threshold::threshold(blue_filtered, front_low_r, front_low_g, front_low_b, front_high_r, front_high_g, front_high_b);
        image_thresholded = vision_commons::Morph::close(image_thresholded, 2*front_closing_mat_point+1, front_closing_mat_point, front_closing_mat_point, front_closing_iter);
        thresholded_front_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());

        cv::Mat image_lines = image_thresholded;
        cv::Canny(convolved, image_lines, front_canny_threshold_low, front_canny_threshold_high);
        lines_front_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_lines).toImageMsg());

        cv::HoughLinesP(image_thresholded, lines, 1, CV_PI/180, front_hough_threshold, front_hough_minline, front_hough_maxgap);

        cv::Point2i coordinate[4], rodB;
        for(int i = 0;i<lines.size(); i++)
        {
          cv::line(image_marked, cv::Point2i(lines[i][0], lines[i][1]), cv::Point2i(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 0), 3, 8, 0);
          if(i==0)
          {
            coordinate[0].x = lines[i][0]; coordinate[0].y = lines[i][1];			//taking y corresponding to min x
            coordinate[1].x = lines[i][2]; coordinate[1].y = lines[i][3];            //taking y corresponding to max x
            coordinate[2].x = lines[i][0]; coordinate[2].y = lines[i][1];            //taking min y
            coordinate[3].x = lines[i][2]; coordinate[3].y = lines[i][3];            //taking max y
          }
          else
          {
            if(lines[i][0] < coordinate[0].x)
              coordinate[0].x = lines[i][0]; coordinate[0].y = lines[i][1];
            if(lines[i][2] > coordinate[1].x)
              coordinate[1].x = lines[i][2]; coordinate[1].y = lines[i][3];
            if(lines[i][0] > coordinate[1].x)
              coordinate[1].x = lines[i][0]; coordinate[1].y = lines[i][1];
            if(lines[i][2] < coordinate[0].x)
              coordinate[0].x = lines[i][2]; coordinate[0].y = lines[i][3];
            if(lines[i][1] < coordinate[2].y)
              coordinate[2].x = lines[i][0]; coordinate[2].y = lines[i][1];
            if(lines[i][3] > coordinate[3].y)
              coordinate[3].x = lines[i][2]; coordinate[3].y = lines[i][3];
            if(lines[i][1] > coordinate[3].y)
              coordinate[3].x = lines[i][0]; coordinate[3].y = lines[i][1];
            if(lines[i][3] < coordinate[2].y)
              coordinate[2].x = lines[i][2]; coordinate[2].y = lines[i][3];
          }
        }
        float angle = angleWrtY(coordinate[0], coordinate[2]);
        if((angle < 25) ||(angle > 155) ) // Check the angle range
          coordinate[2] = rotatePoint(coordinate[0], coordinate[2], -CV_PI/4);

        cv::line(image_marked, coordinate[0], coordinate[2], cv::Scalar(255,0,0), 3, 8);

        geometry_msgs::PointStamped gate_point_message;
        gate_point_message.header.stamp = ros::Time();
        gate_point_message.header.frame_id = camera_frame.c_str();
        gate_point_message.point.x = 0.0;
        gate_point_message.point.y = (coordinate[0].x + coordinate[2].x)/2;
        gate_point_message.point.z = (coordinate[0].y + coordinate[2].y)/2;

        ROS_INFO("Gate Center (y, z) = (%.2f, %.2f)", gate_point_message.point.y, gate_point_message.point.z);
        coordinates_front_pub.publish(gate_point_message);

        cv_bridge::CvImage marked_ptr;
        marked_ptr.header = msg->header;
        marked_ptr.encoding = sensor_msgs::image_encodings::BGR8;
        marked_ptr.image = image_marked;
        marked_front_pub.publish(marked_ptr.toImageMsg());
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

void imageBottomCallback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_img_ptr->image;
    cv::Mat image_marked = cv_img_ptr->image;
    if(!image.empty()){
      cv::Mat blue_filtered = vision_commons::Filter::blue_filter(image, bottom_clahe_clip, bottom_clahe_grid_size, bottom_clahe_bilateral_iter, bottom_balanced_bilateral_iter, bottom_denoise_h);
      blue_filtered_bottom_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", blue_filtered).toImageMsg());
      cv::Mat image_thresholded;
      if(!(front_high_b<=front_low_b || front_high_g<=front_low_g || front_high_r<=front_low_r)) {
        image_thresholded = vision_commons::Threshold::threshold(image, bottom_low_r, bottom_low_g, bottom_low_b, bottom_high_r, bottom_high_g, bottom_high_b);
        image_thresholded = vision_commons::Morph::close(image_thresholded, 2*bottom_closing_mat_point+1, bottom_closing_mat_point, bottom_closing_mat_point, bottom_closing_iter);
        cv_bridge::CvImage thresholded_ptr;
        thresholded_ptr.header = msg->header;
        thresholded_ptr.encoding = sensor_msgs::image_encodings::MONO8;
        thresholded_ptr.image = image_thresholded;
        thresholded_bottom_pub.publish(thresholded_ptr.toImageMsg());
        std::vector<std::vector<cv::Point> > contours = vision_commons::Contour::getBestX(image_thresholded, 1);
        if(contours.size()!=0){
          cv::RotatedRect bounding_rectangle = cv::minAreaRect(cv::Mat(contours[0]));
          geometry_msgs::PointStamped pipe_point_message;
          pipe_point_message.header.stamp = ros::Time();
          pipe_point_message.header.frame_id = camera_frame.c_str();
          pipe_point_message.point.x = (image.size().height)/2 - bounding_rectangle.center.y;
          pipe_point_message.point.y = bounding_rectangle.center.x - (image.size().width)/2;
          pipe_point_message.point.z = 0.0;
          coordinates_bottom_pub.publish(pipe_point_message);
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
          marked_bottom_pub.publish(marked_ptr.toImageMsg());
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
  blue_filtered_front_pub = it.advertise("/gate_task/front/blue_filtered", 1);
  thresholded_front_pub = it.advertise("/gate_task/front/thresholded", 1);
  convolved_front_pub = it.advertise("/gate_task/front/convolved", 1);
  lines_front_pub = it.advertise("/gate_task/front/lines",1);
  marked_front_pub = it.advertise("/gate_task/front/marked", 1);
  coordinates_front_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/front/gate_coordinates", 1000);
  blue_filtered_bottom_pub = it.advertise("/gate_task/bottom/blue_filtered", 1);
  thresholded_bottom_pub = it.advertise("/gate_task/bottom/thresholded", 1);
  marked_bottom_pub = it.advertise("/gate_task/bottom/marked", 1);
  coordinates_bottom_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/bottom/pipe_coordinates", 1000);
  task_done_pub = nh.advertise<std_msgs::Bool>("/gate_task/done", 1000);
  image_transport::Subscriber front_image_sub = it.subscribe("/front_camera/image_raw", 1, imageFrontCallback);
  image_transport::Subscriber bottom_image_sub = it.subscribe("/bottom_camera/image_raw", 1, imageBottomCallback);
  ros::spin();
  return 0;
}
