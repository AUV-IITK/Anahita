// Copyright 2017 AUV-IITK
#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

#include "lib.hpp"
#include <task_buoy/buoyConfig.h>

namespace pre_processing{
  cv::Mat balance_white(cv::Mat src, float parameter); // for removing the color cast
  cv::Mat color_correction(cv::Mat src, int parameter); // apply the CLAHE algorithm here
  void denoise(cv::Mat &src, int i); // applying bilateralFilter 2*i times
};

namespace post_processing{
  std_msgs::Float64MultiArray empty_contour_handler(cv::Point2f); // gives the last coordinates of the center of the buoy if the present frame is empty
  std_msgs::Float64MultiArray edge_case_handler(cv::Point2f, int radius); // tells if buoy is at the edges and which edge
  int get_largest_contour_index(std::vector<std::vector<cv::Point2f> > contours); // to get the biggest contour in a set of contours
  cv::Point2f get_contour_center(std::vector<std::vector<cv::Point2f> > contours); // to get the center of the contour
  void avg_fn(float *radius_ideal, std::vector<cv::Point2f> &center_ideal, float &current_radius, cv::Point2f &current_center, int &count_avg);
};

namespace task_buoy{
  void get_buoys_params(ros::NodeHandle &nh, int r[][2], int b[][2], int g[][2]); // get the parameter values from the parameter server running in the ros master
  void threshold(int BGR[][2], int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2], int flag); // for thresholding three buoys without using 18 variables
  void set_buoy_params(ros::NodeHandle n, int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2]); // for setting the parameter values in the parameter server
  void update_values(task_buoy::buoyConfig &config, int flag, int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2]);
  void threshold_values_update(int BGR[][2], task_buoy::buoyConfig &config);
};

#endif
