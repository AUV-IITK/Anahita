#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <bits/stdc++.h>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <std_msgs/Bool.h>
#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>
#include <vision_commons/geometry.h>

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
int low_h= 0;
int high_h = 255;
int low_s = 0;
int high_s = 255;
int low_v = 0;
int high_v = 255;
int closing_mat_point = 1;
int closing_iter = 1;
int canny_threshold_low = 0;
int canny_threshold_high = 1000;
int canny_kernel_size = 3;
int hough_threshold = 0;
int hough_minline = 0;
int hough_maxgap = 0;
double hough_angle_tolerance = 0.0;
double gate_distance_tolerance = 50.0;
double gate_angle_tolerance = 0.0;
image_transport::Publisher blue_filtered_pub;
image_transport::Publisher thresholded_pub;
image_transport::Publisher canny_pub;
image_transport::Publisher lines_pub;
image_transport::Publisher marked_pub;
ros::Publisher coordinates_pub;

std::string camera_frame = "auv-iitk";

cv::Point2i rotatePoint(const cv::Point2i &v1, const cv::Point2i &v2, float angle) {
  if(v1.x > v2.x) {
    cv::Point2i v3 = v1 - v2;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v2;
    return finalVertex;
  }
  else {
    cv::Point2i v3 = v2 - v1;
    cv::Point2i finalVertex;
    finalVertex.x = v3.x * cos(angle) - v3.y * sin(angle);
    finalVertex.y = v3.x * sin(angle) + v3.y * cos(angle);
    finalVertex = finalVertex + v1;
    return finalVertex;
  }
}

void callback(vision_tasks::gateFrontRangeConfig &config, double level){
  clahe_clip = config.clahe_clip;
  clahe_grid_size = config.clahe_grid_size;
  clahe_bilateral_iter = config.clahe_bilateral_iter;
  balanced_bilateral_iter = config.balanced_bilateral_iter;
  denoise_h = config.denoise_h;
  low_h = config.low_h;
  high_h = config.high_h;
  low_s = config.low_s;
  high_s = config.high_s;
  low_v = config.low_v;
  high_v = config.high_v;
  closing_mat_point = config.closing_mat_point;
  closing_iter = config.closing_iter;
  canny_threshold_low = config.canny_threshold_low;
  canny_threshold_high = config.canny_threshold_high;
  canny_kernel_size = config.canny_kernel_size;
  hough_threshold = config.hough_threshold;
  hough_minline = config.hough_minline;
  hough_maxgap = config.hough_maxgap;
  hough_angle_tolerance = config.hough_angle_tolerance;
  gate_distance_tolerance = config.gate_distance_tolerance;
  gate_angle_tolerance = config.gate_angle_tolerance;
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
      cv::Mat hsv;
      cv::cvtColor(blue_filtered, hsv, CV_BGR2HSV);
      cv::Mat image_thresholded = vision_commons::Threshold::threshold(hsv, low_h, low_s, low_v, high_h, high_s, high_v);
      image_thresholded = vision_commons::Morph::close(image_thresholded, 2*closing_mat_point+1, closing_mat_point, closing_mat_point, closing_iter);
      thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());
      cv::Mat image_gray;
      cv::cvtColor(image_thresholded, image_gray, CV_GRAY2BGR);
      cv::Mat image_canny;
      cv::Canny(image_gray, image_canny, canny_threshold_low, canny_threshold_high, canny_kernel_size);
      canny_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_canny).toImageMsg());
      cv::Mat image_lines = blue_filtered;
      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(image_thresholded, lines, 1, CV_PI/180, hough_threshold, hough_minline, hough_maxgap);
      std::vector<cv::Vec4i> lines_filtered;
      std::vector<double> angles;
      double angle = 0.0;
      for(int i = 0 ; i < lines.size() ; i++) {
        cv::Point p1(lines[i][0], lines[i][1]);
        cv::Point p2(lines[i][2], lines[i][3]);
        angle = vision_commons::Geometry::angleWrtY(p1, p2);
        if(angle < hough_angle_tolerance || abs(angle - 90.0) < hough_angle_tolerance || abs(180.0 - angle) < hough_angle_tolerance) {
          cv::line(image_lines, p1, p2, cv::Scalar(0, 120, 255), 3, CV_AA);
          lines_filtered.push_back(lines[i]);
          angles.push_back(angle);
        }
        else {
          cv::line(image_lines, p1, p2, cv::Scalar(255, 120, 0), 3, CV_AA);
        }
      }
      lines_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_lines).toImageMsg());
      int found = 0;
      cv::Point2i pi1;
      cv::Point2i pi2;
      cv::Point2i pj1;
      cv::Point2i pj2;
      cv::Point2i horizontal1(0, 0);
      cv::Point2i horizontal2(0, 0);
      cv::Point2i vertical1(0, 0);
      cv::Point2i vertical2(0, 0);
      cv::Point2i longest1(0, 0);
      cv::Point2i longest2(0, 0);
      for(int i = 0 ; i < lines_filtered.size() ; i++) {
        pi1.x = lines_filtered[i][0];
        pi1.y = lines_filtered[i][1];
        pi2.x = lines_filtered[i][2];
        pi2.y = lines_filtered[i][3];
        if(vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(longest1, longest2)) {
          longest1.x = pi1.x;
          longest1.y = pi1.y;
          longest2.x = pi2.x;
          longest2.y = pi2.y;
        }
        for(int j = i+1 ; j < lines_filtered.size() ; j++) {
          pj1.x = lines_filtered[j][0];
          pj1.y = lines_filtered[j][1];
          pj2.x = lines_filtered[j][2];
          pj2.y = lines_filtered[j][3];
          if(abs(angles[i]-angles[j] - 90.0) < gate_angle_tolerance) {
            double distance1 = vision_commons::Geometry::distance(pi1, pj1);
            double distance2 = vision_commons::Geometry::distance(pi1, pj2);
            double distance3 = vision_commons::Geometry::distance(pi2, pj1);
            double distance4 = vision_commons::Geometry::distance(pi2, pj2);
            if(distance1 < gate_distance_tolerance || distance2 < gate_distance_tolerance || distance3 < gate_distance_tolerance || distance4 < gate_distance_tolerance) {
              if(abs(angles[j] - 90.0) < abs(angles[i] - 90.0) && (vision_commons::Geometry::distance(pj1, pj2) > vision_commons::Geometry::distance(horizontal1, horizontal2) || vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(vertical1, vertical2))) {
                horizontal1.x = pj1.x;
                horizontal1.y = pj1.y;
                horizontal2.x = pj2.x;
                horizontal2.y = pj2.y;
                vertical1.x = pi1.x;
                vertical1.y = pi1.y;
                vertical2.x = pi2.x;
                vertical2.y = pi2.y;
                found = 1;
              }
              else if(abs(angles[j] - 90.0) > abs(angles[i] - 90.0) && (vision_commons::Geometry::distance(pi1, pi2) > vision_commons::Geometry::distance(horizontal1, horizontal2) || vision_commons::Geometry::distance(pj1, pj2) > vision_commons::Geometry::distance(vertical1, vertical2))) {
                horizontal1.x = pi1.x;
                horizontal1.y = pi1.y;
                horizontal2.x = pi2.x;
                horizontal2.y = pi2.y;
                vertical1.x = pj1.x;
                vertical1.y = pj1.y;
                vertical2.x = pj2.x;
                vertical2.y = pj2.y;
                found = 1;
              }
            }
          }
        }
      }
      geometry_msgs::PointStamped gate_point_message;
      gate_point_message.header.stamp = ros::Time();
      gate_point_message.header.frame_id = camera_frame.c_str();
      if(found) {
        gate_point_message.point.x = pow(sqrt(pow(vision_commons::Geometry::distance(horizontal1, horizontal2), 2) + pow(vision_commons::Geometry::distance(vertical1, vertical2), 2))/7526.5, -.92678);
        gate_point_message.point.y = (horizontal1.x + horizontal2.x)/2 - image.size().width/2;
        gate_point_message.point.z = image.size().height/2 - (vertical1.y + vertical2.y)/2;
        ROS_INFO("Gate Center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
        cv::line(image_marked, horizontal1, horizontal2, cv::Scalar(0, 0, 0), 3, CV_AA);
        cv::line(image_marked, vertical1, vertical2, cv::Scalar(255, 255, 255), 3, CV_AA);
      }
      else {
        if(abs(vision_commons::Geometry::angleWrtY(longest1, longest2) - 90.0) < hough_angle_tolerance) {
          gate_point_message.point.x = pow((vision_commons::Geometry::distance(longest1, longest2)*1.068)/7526.5, -.92678);
          gate_point_message.point.y = (longest1.x + longest2.x)/2 - image.size().width/2;
          gate_point_message.point.z = image.size().height/2 - (longest1.y + longest2.y)/2 + 9*vision_commons::Geometry::distance(longest1, longest2)/48;
        }
        else {
          gate_point_message.point.x = pow((vision_commons::Geometry::distance(longest1, longest2)*2.848)/7526.5, -.92678);
          gate_point_message.point.y = (longest1.x + longest2.x)/2 + 12*vision_commons::Geometry::distance(longest1, longest2)/9 - image.size().width/2;
          gate_point_message.point.z = image.size().height/2 - (longest1.y + longest2.y)/2;
        }
        cv::line(image_marked, longest1, longest2, cv::Scalar(0, 0, 255), 3, CV_AA);
        ROS_INFO("Couldn't find gate, estimated gate center (x, y, z) = (%.2f, %.2f, %.2f)", gate_point_message.point.x, gate_point_message.point.y, gate_point_message.point.z);
      }
      coordinates_pub.publish(gate_point_message);
      cv::circle(image_marked, cv::Point(gate_point_message.point.y + image.size().width/2, image.size().height/2 - gate_point_message.point.z), 1, cv::Scalar(0,155,155), 8, 0);
      cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(0,155, 205), 8, 0);
      cv_bridge::CvImage marked_ptr;
      marked_ptr.header = msg->header;
      marked_ptr.encoding = sensor_msgs::image_encodings::BGR8;
      marked_ptr.image = image_marked;
      marked_pub.publish(marked_ptr.toImageMsg());
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
  ros::init(argc, argv, "gate_task_front");
  ros::NodeHandle nh;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig> server;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  image_transport::ImageTransport it(nh);
  blue_filtered_pub = it.advertise("/gate_task/front/blue_filtered", 1);
  thresholded_pub = it.advertise("/gate_task/front/thresholded", 1);
  canny_pub = it.advertise("/gate_task/front/canny", 1);
  lines_pub = it.advertise("/gate_task/front/lines",1);
  marked_pub = it.advertise("/gate_task/front/marked", 1);
  coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/front/gate_coordinates", 1000);
  //image_transport::Subscriber front_image_sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  image_transport::Subscriber front_image_sub = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}
