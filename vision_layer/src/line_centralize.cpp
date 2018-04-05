// Copyright 2017 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <task_line/lineConfig.h>
#include "std_msgs/Float32MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include <std_msgs/Float64MultiArray.h>

bool IP = true;
bool flag = false;
bool video = false;
cv::Mat frame;
cv::Mat newframe;
int count_avg = 0;
int t1min, t1max, t2min, t2max, t3min, t3max;

void callback(task_line::lineConfig &config, double level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("centralize_Reconfigure Request:New params: %d %d %d %d %d %d ", t1min, t1max, t2min, t2max, t3min, t3max);
}

float mod(float x, float y)
{
  if (x - y > 0)
    return x;
  else
    return y;
}
void Switch_callback(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    ///////////////////////////// DO NOT REMOVE THIS, IT COULD BE INGERIOUS TO HEALTH /////////////////////
    newframe.copyTo(frame);
    ////////////////////////// FATAL ///////////////////////////////////////////////////
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void balance_white(cv::Mat mat)
{
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }


  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}


int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on
  cv::Scalar color(255, 255, 255);

  ros::init(argc, argv, "line_centralize");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/line_centralize", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("line_centralize_switch", 1000, &Switch_callback);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/bottom_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<task_line::lineConfig> server;
  dynamic_reconfigure::Server<task_line::lineConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // capture size -
  CvSize size = cvSize(width, height);

  cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst , dst1;  // image converted to HSV plane

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();

    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }


    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    balance_white(frame);
    bilateralFilter(frame, dst1, 4, 8, 8);
    cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);

    cv::inRange(dst1, cv::Scalar(0, 0, 20), cv::Scalar(80, 260, 260), thresholded);

    cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    if ((!IP))
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat;
      thresholded.copyTo(thresholded_Mat);
      cv::findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;

      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();

      pub2.publish(msg2);
      pub3.publish(msg3);

      if (contours.empty())
      {
        array.data.push_back(0);
        array.data.push_back(0);

        pub.publish(array);
        ros::spinOnce();

        continue;
      }

      for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
      {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
          largest_area = a;
          largest_contour_index = i;  // Store the index of largest contour
        }
      }

      // Convex HULL
      cv::Mat Drawing(thresholded.rows, thresholded.cols, CV_8UC1, cv::Scalar::all(0));
      std::vector<std::vector<cv::Point> > hull(1);
      cv::convexHull(cv::Mat(contours[largest_contour_index]), hull[0], false);

      cv::Moments mu;
      std::vector<cv::Vec4i> hierarchy;
      mu = cv::moments(hull[0], false);
      cv::Point2f center_of_mass;

      center_of_mass = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

      cv::drawContours(Drawing, hull, 0, color, 2, 8, hierarchy);
      cv::circle(frame, center_of_mass, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      // cv::imshow("LineCentralize:COM", frame);

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub1.publish(msg1);


      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

      array.data.push_back((320 - center_of_mass.x));
      array.data.push_back((240 - center_of_mass.y));
      pub.publish(array);

      ros::spinOnce();
    }

    else
    {
      ros::spinOnce();
    }
  }
  return 0;
}
