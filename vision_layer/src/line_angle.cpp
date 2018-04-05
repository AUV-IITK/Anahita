// Copyright 2017 AUV-IITK
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <vector>
#include <cv.h>
#include <highgui.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <task_line/lineConfig.h>
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>
#include "std_msgs/Header.h"
using cv::Mat;
using cv::split;
using cv::Size;
using cv::Scalar;
using cv::normalize;
using cv::Point;
using cv::VideoCapture;
using cv::NORM_MINMAX;
using cv::MORPH_ELLIPSE;
using cv::COLOR_BGR2HSV;
using cv::destroyAllWindows;
using cv::getStructuringElement;
using cv::Vec4i;
using cv::namedWindow;
using std::vector;
using std::endl;
using std::cout;

int w = -2, x = -2, y = -2, z = -2;
bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max, lineCount = 0;

void callback_dyn(task_line::lineConfig &config, double level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Line_angle Reconfigure: New params: %d %d %d %d %d %d ", t1min, t1max, t2min, t2max, t3min, t3max);
}

std_msgs::Float64 msg;
// parameters in param file should be nearly the same as the commented values
// params for an orange strip
int LowH = 0;    // 0
int HighH = 88;  // 88

int LowS = 0;     // 0
int HighS = 251;  // 251

int LowV = 0;     // 0
int HighV = 255;  // 255

// params for hough line transform
int lineThresh = 60;     // 60
int minLineLength = 70;  // 70
int maxLineGap = 10;     // 10
int houghThresh = 15;    // 15

double rho = 0.1;
double finalAngle = -1;
double minDeviation = 0.02;

cv::Mat frame;
cv::Mat newframe, sent_to_callback, imgLines;
int count = 0, count_avg = 0;

double computeMean(vector<double> &newAngles)
{
  double sum = 0;
  for (size_t i = 0; i < newAngles.size(); i++)
  {
    sum = sum + newAngles[i];
  }
  return sum / newAngles.size();
}
// called when few lines are detected
// to remove errors due to any stray results
double computeMode(vector<double> &newAngles)
{
  double mode = newAngles[0];
  int freq = 1;
  int tempFreq;
  double diff;
  for (int i = 0; i < newAngles.size(); i++)
  {
    tempFreq = 1;

    for (int j = i + 1; j < newAngles.size(); j++)
    {
      diff = newAngles[j] - newAngles[i] > 0.0 ? newAngles[j] - newAngles[i] : newAngles[i] - newAngles[j];
      if (diff <= minDeviation)
      {
        tempFreq++;
        newAngles.erase(newAngles.begin() + j);
        j = j - 1;
      }
    }

    if (tempFreq >= freq)
    {
      mode = newAngles[i];
      freq = tempFreq;
    }
  }

  return mode;
}

void callback(int, void *)
{
  vector<Vec4i> lines;
  HoughLinesP(sent_to_callback, lines, 1, CV_PI / 180, lineThresh, minLineLength, maxLineGap);

  frame.copyTo(imgLines);
  imgLines = Scalar(0, 0, 0);
  vector<double> angles(lines.size());

  lineCount = lines.size();
  int j = 0;
  for (size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];
    line(imgLines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, CV_AA);
    if ((l[2] == l[0]) || (l[1] == l[3]))
      continue;
    angles[j] = atan(static_cast<double>(l[2] - l[0]) / (l[1] - l[3]));
    j++;
  }

  imshow("LineAngle:LINES", imgLines + frame);

  // if num of lines are large than one or two stray lines won't affect the mean
  // much
  // but if they are small in number than mode has to be taken to save the error
  // due to those stray line

  if (lines.size() > 0 && lines.size() < 10)
    finalAngle = computeMode(angles);
  else if (lines.size() > 0)
    finalAngle = computeMean(angles);
}

void lineAngleListener(std_msgs::Bool msg)
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

  ros::init(argc, argv, "line_angle");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float64>("/varun/ip/line_angle", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("line_angle_switch", 1000, &lineAngleListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/bottom_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<task_line::lineConfig> server;
  dynamic_reconfigure::Server<task_line::lineConfig>::CallbackType f;
  f = boost::bind(&callback_dyn, _1, _2);
  server.setCallback(f);

  // capture size -
  CvSize size = cvSize(width, height);

  cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst;

  // Initialize different images that are going to be used in the program
  cv::Mat red_hue_image, dst1;  // image converted to HSV plane
  // asking for the minimum distance where bwe fire torpedo

  while (ros::ok())
  {
    loop_rate.sleep();
    // Get one frame
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

    cv::inRange(dst1, cv::Scalar(0, 0, 20), cv::Scalar(80, 260, 260), red_hue_image);

    cv::erode(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    cv::imshow("LineAngle:AfterThresholding", red_hue_image);  // The stream after color filtering

    if (!IP)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = red_hue_image;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
      double largest_area = 0, largest_contour_index = 0;

      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();

      pub2.publish(msg2);
      pub3.publish(msg3);

      if (contours.empty())
      {
        msg.data = -finalAngle * (180 / 3.14) + 90;
        pub.publish(msg);
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
      std::vector<std::vector<cv::Point> > hull(contours.size());
      convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
      std::vector<cv::Vec4i> hierarchy;
      cv::Scalar color(255, 255, 255);
      drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);
      // cv::imshow("LineAngle:Contours", Drawing);

      std_msgs::Float64 msg;
      Drawing.copyTo(sent_to_callback);
      /*
      msg.data never takes positive 90
      when the angle is 90 it will show -90
      -------------TO BE CORRECTED-------------
      */

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Drawing).toImageMsg();
      pub1.publish(msg1);

      msg.data = -finalAngle * (180 / 3.14);
      if (lineCount > 0)
      {
        pub.publish(msg);
      }
      callback(0, 0);  // for displaying the thresholded image initially
      ros::spinOnce();
      loop_rate.sleep();

      ros::spinOnce();
    }
    else
    {
      ros::spinOnce();
    }
  }

  return 0;
}
