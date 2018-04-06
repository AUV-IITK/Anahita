// Copyright 2017 AUV-IITK
#include "lib.hpp"
#include <task_buoy/buoyConfig.h>
#include "pre_processing.h"

#define SHELLSCRIPT_DUMP "\
#/bin/bash \n\
echo -e \"parameters dumped!!\" \n\
rosparam dump ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"
#define SHELLSCRIPT_LOAD "\
#/bin/bash \n\
echo -e \"parameters loaded!!\" \n\
rosparam load ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"

bool IP = true;
bool save = false;
bool threshold = false;
int flag = 0;
int count = 0;
int BGR[3][2];  // rgb values for color filtering

int red_buoy[3][2], green_buoy[3][2], blue_buoy[3][2]; // rows for colors and colors for max and min ; 0, 1, 2 for blue, green and red ; 0 for min and 1 max
cv::Mat frame;
cv::Mat newframe;
cv::Mat dst;

int count_avg = 0;

void callback(task_buoy::buoyConfig &config, uint32_t level)
{
  flag = config.flag_param; // flag parameter for thresholding the buoys
  threshold = config.threshold_param; // initial value is false

  if (!threshold){
    task_buoy::update_values(config, flag, blue_buoy, green_buoy, red_buoy); // update the values in the rqt_reconfigure which were saved last time
  }

  // updating the values in the BGR matrix
  task_buoy::threshold_values_update(BGR, config);

  if (!count){
    config.save_param = false;
    count++;
  }

  save = config.save_param;

  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d %d %d %d", BGR[0][0], BGR[0][1], BGR[1][0], BGR[1][1], BGR[2][0], BGR[2][1], save, flag, threshold);
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
    newframe.copyTo(frame);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "buoy_detection");
  int height, width, step, channels;  // parameters of the image we are working on
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("buoy_detection_switch", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  system(SHELLSCRIPT_LOAD);

  task_buoy::get_buoys_params(n, red_buoy, blue_buoy, green_buoy); // to get the values of parameters from the parameter server

  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;

  cv::Mat lab_image, balanced_image1, dstx, image_clahe, dst, dst1;
  std::vector<cv::Mat> buoys(3);
  std::vector<cv::Mat> thresholded(3);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();

    if (! ros::param::has("/buoy_detection/b2min")) // for checking whether the parameters are created or not
      std::cout << "not exist" << std::endl;

    if (threshold){
      task_buoy::threshold(BGR, red_buoy, blue_buoy, green_buoy, flag); // to put change the values of red_buoy, green_buoy, blue_buoy through dynamic_reconfigure
    }

    if (save == true){

      task_buoy::set_buoy_params(n, red_buoy, blue_buoy, green_buoy);
      save = false;
      system(SHELLSCRIPT_DUMP); // all the parameters saved in the yaml file

    }

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

    image_clahe = pre_processing::color_correction(frame, 4);

    pre_processing::denoise(image_clahe, 7); // for removing the colored noise

    image_clahe.copyTo(balanced_image1);
    dst1 = pre_processing::balance_white(balanced_image1, 0.05);

    pre_processing::denoise(dst1, 2);

    for (int i = 0; i < 3; i++)
      buoys[i] = dst1;

    cv::Scalar red_buoy_min = cv::Scalar(red_buoy[0][0], red_buoy[1][0], red_buoy[2][0], 0);
    cv::Scalar red_buoy_max = cv::Scalar(red_buoy[0][1], red_buoy[1][1], red_buoy[2][1], 0);

    cv::Scalar blue_buoy_min = cv::Scalar(blue_buoy[0][0], blue_buoy[1][0], blue_buoy[2][0], 0);
    cv::Scalar blue_buoy_max = cv::Scalar(blue_buoy[0][1], blue_buoy[1][1], blue_buoy[2][1], 0);

    cv::Scalar green_buoy_min = cv::Scalar(green_buoy[0][0], green_buoy[1][0], green_buoy[2][0], 0);
    cv::Scalar green_buoy_max = cv::Scalar(green_buoy[0][1], green_buoy[1][1], green_buoy[2][1], 0);

    // thresholding all the colors according to their thresholding values
    cv::inRange(buoys[0], red_buoy_min, red_buoy_max, thresholded[0]);
    cv::inRange(buoys[1], green_buoy_min, green_buoy_max, thresholded[1]);
    cv::inRange(buoys[2], blue_buoy_min, blue_buoy_max, thresholded[2]);

    // Filter out colors which are out of range.

    for (int i = 0; i < 3; i++)
    {
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    }

    if (1)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded[0];
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      std::cout << "inside if" << std::endl;
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded_Mat).toImageMsg();

      pub2.publish(msg2);
      pub3.publish(msg3);

      if (contours.empty())
      {
        array = post_processing::empty_contour_handler(center_ideal[0]);
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
      std::vector<std::vector<cv::Point> > hull(contours.size());
      convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      std::vector<cv::Point2f> center(1);
      std::vector<float> radius(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);
      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

      float r_avg = (r[0] + r[1] + r[2] + r[3] + r[4]) / 5;
      if ((radius[0] < (r_avg + 10)) && (count_avg >= 5))
      {
        r[4] = r[3];
        r[3] = r[2];
        r[2] = r[1];
        r[1] = r[0];
        r[0] = radius[0];
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = center[0];
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        r[count_avg] = radius[0];
        center_ideal[count_avg] = center[0];
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", circles).toImageMsg();
      pub1.publish(msg1);

      array = post_processing::edge_case_handler(center_ideal[0], r[0]);
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
