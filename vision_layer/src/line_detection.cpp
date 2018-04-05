// Copyright 2016 AUV-IITK
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
#include <dynamic_reconfigure/server.h>
#include <task_line/lineConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int percentage = 5; // used for how much percent of the screen should be orange
                             // before deciding that a line is below. Used in
                             // dynamic_reconfig
// callback for change the percent of orange before saying there is a line below
bool IP = false;
bool flag = false;
bool video = false;
cv::Mat red_hue_image;
cv::Mat frame;
cv::Mat newframe;
int count = 0;

void callback(task_line::lineConfig &config, uint32_t level)
{
  percentage = config.orange_param;
  ROS_INFO("%s Reconfigure Request : New parameters :%d", ros::this_node::getName().c_str(), percentage);
}

void lineDetectedListener(std_msgs::Bool msg)
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


// callback for off switch.
int detect(cv::Mat image)
{
  cv::Size size(640, 480);  // the dst image size,e.g.100x100
  cv::Mat resizeimage;      // dst image
  cv::Mat bgr_image;
  cv::Mat dst1;
  resize(image, resizeimage, size);  // resize image
  cv::waitKey(20);
  // detect red color here
  balance_white(resizeimage);
  bilateralFilter(resizeimage, dst1, 4, 8, 8);

  cv::inRange(dst1, cv::Scalar(0, 0, 20), cv::Scalar(80, 260, 260), red_hue_image);

  cv::erode(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::erode(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(red_hue_image, red_hue_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  int nonzero = countNonZero(red_hue_image);
  int nonzeropercentage = nonzero / 3072;
  if (nonzero > (3072 * percentage))  // return 1 if a major portion of image
                                      // has red color, Note : here the size of
                                      // image is 640X480 = 307200.
    return 1;
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");
  ros::NodeHandle n;
  ros::Publisher robot_pub = n.advertise<std_msgs::Bool>("/varun/ip/line_detection", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("line_detection_switch", 1000, &lineDetectedListener);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/bottom_camera/image_raw", 1, imageCallback);
  ros::Rate loop_rate(12);  // this rate should be same as the rate of camera
                            // input. and in the case of other sensors , this
                            // rate should be same as there rate of data
                            // generation
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<task_line::lineConfig> server;
  dynamic_reconfigure::Server<task_line::lineConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  n.getParam("line_detection/percentage", percentage);

  task_line::lineConfig config;
  config.orange_param = percentage;
  callback(config, 0);

  int oldAlert = 42;  // Used to decide when to print

  while (ros::ok())
  {
    if (!frame.data)  // Check for invalid input
    {
      ROS_INFO("%s: Could not open or find the image\n", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    if (1)
    {
      int alert = detect(frame);

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_hue_image).toImageMsg();
      // sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      // sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();

      pub1.publish(msg1);
      // pub2.publish(msg2);
      // pub3.publish(msg3);


      if (alert == 1)
      {
        std_msgs::Bool msg;
        msg.data = true;
        robot_pub.publish(msg);
        if (oldAlert != alert)
        {
          ROS_INFO("%s: found line", ros::this_node::getName().c_str());
          oldAlert = alert;
        }
      }
      else if (alert == 0)
      {
        std_msgs::Bool msg;
        msg.data = false;
        robot_pub.publish(msg);
        if (oldAlert != alert)
        {
          ROS_INFO("%s: no line", ros::this_node::getName().c_str());
          oldAlert = alert;
        }
      }
      else
      {
        return 0;
      }
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
