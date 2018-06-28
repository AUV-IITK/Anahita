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
#include <std_msgs/Bool.h>
#include <vision_tasks/gateFrontRangeConfig.h>
#include <vision_commons/morph.h>
#include <vision_commons/contour.h>
#include <vision_commons/threshold.h>
#include <vision_commons/filter.h>

//cv::Mat horizontalkern = (cv::Mat_<float>(3,3) << 1,1,1,0,0,0,1,1,1)/(float)(9);
//cv::Mat verticalkern = (cv::Mat_<float>(3,3) << 1,0,1,1,0,1,1,0,1)/(float)(9);
cv::Mat kern = (cv::Mat_<float>(3,3) << 1, 0, 0, 1, 0, 0, 1, 1, 1)/(float)(9);

double clahe_clip = 4.0;
int clahe_grid_size = 8;
int clahe_bilateral_iter = 8;
int balanced_bilateral_iter = 4;
double denoise_h = 10.0;
int canny_threshold_low = 0;
int canny_threshold_high = 0;
int hough_threshold = 0;
int hough_minline = 0;
int hough_maxgap = 0;
int low_b = 10;
int low_g = 0;
int low_r = 0;
int high_b = 90;
int high_g= 255;
int high_r = 255;
int closing_mat_point = 1;
int closing_iter = 1;

cv::SimpleBlobDetector::Params params;
cv::Ptr<cv::SimpleBlobDetector> detector;
image_transport::Publisher blue_filtered_pub;
//image_transport::Publisher thresholded_pub;
//image_transport::Publisher convolved_pub;
image_transport::Publisher blobs_pub;
image_transport::Publisher marked_pub;
//image_transport::Publisher lines_pub;
ros::Publisher coordinates_pub;

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


void callback(vision_tasks::gateFrontRangeConfig &config, double level){
  clahe_clip = config.clahe_clip;
  clahe_grid_size = config.clahe_grid_size;
  clahe_bilateral_iter = config.clahe_bilateral_iter;
  balanced_bilateral_iter = config.balanced_bilateral_iter;
  denoise_h = config.denoise_h;
  canny_threshold_low = config.canny_threshold_low;
  canny_threshold_high = config.canny_threshold_high;
  hough_threshold = config.hough_threshold;
  hough_minline = config.hough_minline;
  hough_maxgap = config.hough_maxgap;
  low_b = config.low_b;
  low_g = config.low_g;
  low_r = config.low_r;
  high_b = config.high_b;
  high_g = config.high_g;
  high_r = config.high_r;
  closing_mat_point = config.closing_mat_point;
  closing_iter = config.closing_iter;

  params.filterByArea = config.filterByArea;
  params.filterByCircularity = config.filterByCircularity;
  params.filterByColor = config.filterByColor;
  params.filterByConvexity = config.filterByConvexity;
  params.filterByInertia = config.filterByInertia;
  params.minThreshold = config.minThreshold;
  params.maxThreshold = config.maxThreshold;
  params.thresholdStep = config.thresholdStep;
  params.minArea = config.minArea;
  params.maxArea = config.maxArea;
  params.minCircularity = config.minCircularity;
  params.maxCircularity = config.maxCircularity;
  params.minConvexity = config.minConvexity;
  params.maxConvexity = config.maxConvexity;
  params.minInertiaRatio = config.minInertiaRatio;
  params.maxInertiaRatio = config.maxInertiaRatio;
  params.minDistBetweenBlobs = config.minDistBetweenBlobs;
  detector = cv::SimpleBlobDetector::create(params);
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
      if(canny_threshold_high > canny_threshold_low) {
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
        //std::vector<cv::Mat> planes(3);
        //cv::split(blue_filtered, planes);
        //for(int j = 0 ; j < 3 ; j++) {
        //for(int i = 0 ; i < 3 ; i++) cv::filter2D(planes[j], planes[j], planes[j].depth(), kern);
        //}
        //cv::Mat convolved;
        //merge(planes, convolved);
        //cv::addWeighted(horizontal, 0.5, vertical, 0.5, 0.0,  convolved);
        //cv::Mat copy;
        //convolved.copyTo(copy);
        //copy.convertTo(convolved, -1, 5.0, 0.0);
        //convolved = vision_commons::Filter::clahe(convolved, 4.0, 11);
        //convolved_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", convolved).toImageMsg());
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(blue_filtered, keypoints);
        if(keypoints.size() > 0) {
          cv::Mat image_keypoints;
          cv::drawKeypoints(blue_filtered, keypoints, image_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          blobs_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_keypoints).toImageMsg());
          int iter = keypoints.size();
          if(iter > 20) iter = 20;
          float max_size = 0.0f;
          int max_index = 0;
          float size = 0.0f;
          for(int i = 0 ; i < iter ; i++) {
            max_size = keypoints[i].size;
            max_index = i;
            for(int j = i+1 ; j < keypoints.size() ; j++) {
              size = keypoints[j].size;
              if(size > max_size) {
                max_size = size;
                max_index = j;
              }
            }
            cv::KeyPoint temp = keypoints[i];
            keypoints[i] = keypoints[max_index];
            keypoints[max_index] = temp;
            max_size = 0.0;
            max_index = 0;
            size = 0.0;
          }
          //std::vector<cv::Vec4i> lines;
          //cv::Mat image_thresholded = vision_commons::Threshold::threshold(blue_filtered, low_r, low_g, low_b, high_r, high_g, high_b);
          //image_thresholded = vision_commons::Morph::close(image_thresholded, 2*closing_mat_point+1, closing_mat_point, closing_mat_point, closing_iter);
          //thresholded_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_thresholded).toImageMsg());

          //cv::Mat image_lines = image_thresholded;
          //cv::Canny(convolved, image_lines, canny_threshold_low, canny_threshold_high);
          //lines_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", image_lines).toImageMsg());

          //cv::HoughLinesP(image_thresholded, lines, 1, CV_PI/180, hough_threshold, hough_minline, hough_maxgap);

          //cv::Point2i coordinate[4], rodB;
          //for(int i = 0;i<lines.size(); i++)
          //{
          /*
           *          cv::line(image_marked, cv::Point2i(lines[i][0], lines[i][1]), cv::Point2i(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 0), 3, 8, 0);
           *          if(i==0)
           *          {
           *            coordinate[0].x = lines[i][0]; coordinate[0].y = lines[i][1];//taking y corresponding to min x
           *            coordinate[1].x = lines[i][2]; coordinate[1].y = lines[i][3];//taking y corresponding to max x
           *            coordinate[2].x = lines[i][0]; coordinate[2].y = lines[i][1];//taking min y
           *            coordinate[3].x = lines[i][2]; coordinate[3].y = lines[i][3];//taking max y
           *          }
           *          else
           *          {
           *            if(lines[i][0] < coordinate[0].x)
           *              coordinate[0].x = lines[i][0]; coordinate[0].y = lines[i][1];
           *            if(lines[i][2] > coordinate[1].x)
           *              coordinate[1].x = lines[i][2]; coordinate[1].y = lines[i][3];
           *            if(lines[i][0] > coordinate[1].x)
           *              coordinate[1].x = lines[i][0]; coordinate[1].y = lines[i][1];
           *            if(lines[i][2] < coordinate[0].x)
           *              coordinate[0].x = lines[i][2]; coordinate[0].y = lines[i][3];
           *            if(lines[i][1] < coordinate[2].y)
           *              coordinate[2].x = lines[i][0]; coordinate[2].y = lines[i][1];
           *            if(lines[i][3] > coordinate[3].y)
           *              coordinate[3].x = lines[i][2]; coordinate[3].y = lines[i][3];
           *            if(lines[i][1] > coordinate[3].y)
           *              coordinate[3].x = lines[i][0]; coordinate[3].y = lines[i][1];
           *            if(lines[i][3] < coordinate[2].y)
           *              coordinate[2].x = lines[i][2]; coordinate[2].y = lines[i][3];
           *          }
           *        }
           *        float angle = angleWrtY(coordinate[0], coordinate[2]);
           *        if((angle < 25) ||(angle > 155) ) // Check the angle range
           *          coordinate[2] = rotatePoint(coordinate[0], coordinate[2], -CV_PI/4);
           *
           *        cv::line(image_marked, coordinate[0], coordinate[2], cv::Scalar(255,0,0), 3, 8);
           */
          float y_total = 0.0f;
          float z_total = 0.0f;
          float total = 0.0f;
          for(int i = 0 ; i < iter ; i++) {
            y_total += keypoints[i].pt.x * keypoints[i].size;
            z_total += keypoints[i].pt.y * keypoints[i].size;
            total += keypoints[i].size;
          }
          geometry_msgs::PointStamped gate_point_message;
          gate_point_message.header.stamp = ros::Time();
          gate_point_message.header.frame_id = camera_frame.c_str();
          gate_point_message.point.x = 0.0;
          //gate_point_message.point.y = (coordinate[0].x + coordinate[2].x)/2;
          //gate_point_message.point.z = (coordinate[0].y + coordinate[2].y)/2;
          gate_point_message.point.y = y_total/total - image.size().width/2;
          gate_point_message.point.z = image.size().height/2 - z_total/total;
          ROS_INFO("Gate Center (y, z) = (%.2f, %.2f)", gate_point_message.point.y, gate_point_message.point.z);
          coordinates_pub.publish(gate_point_message);
          cv::circle(image_marked, cv::Point(y_total/total, z_total/total), 1, cv::Scalar(0,155,155), 8, 0);
          cv::circle(image_marked, cv::Point(image.size().width/2, image.size().height/2), 1, cv::Scalar(0,155, 205), 8, 0);

          cv_bridge::CvImage marked_ptr;
          marked_ptr.header = msg->header;
          marked_ptr.encoding = sensor_msgs::image_encodings::BGR8;
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
  ros::init(argc, argv, "gate_task_front");
  ros::NodeHandle nh;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig> server;
  dynamic_reconfigure::Server<vision_tasks::gateFrontRangeConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  image_transport::ImageTransport it(nh);
  blue_filtered_pub = it.advertise("/gate_task/front/blue_filtered", 1);
  //thresholded_pub = it.advertise("/gate_task/front/thresholded", 1);
  //convolved_pub = it.advertise("/gate_task/front/convolved", 1);
  blobs_pub = it.advertise("/gate_task/front/blobs", 1);
  //lines_pub = it.advertise("/gate_task/front/lines",1);
  marked_pub = it.advertise("/gate_task/front/marked", 1);
  coordinates_pub = nh.advertise<geometry_msgs::PointStamped>("/gate_task/front/gate_coordinates", 1000);
  image_transport::Subscriber front_image_sub = it.subscribe("/front_camera/image_raw", 1, imageCallback);
  ros::spin();
  return 0;
}
