// Copyright 2017 AUV-IITK
#include "lib.hpp"
#include <task_buoy/buoyConfig.h>

namespace pre_processing{

  cv::Mat balance_white(cv::Mat src, float parameter){ // same for all the tasks

    float discard_ratio = parameter;
    cv::Mat mat = src;

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

    return mat;
  }

///////////////////////////////////////////////////////////////////////////////////////////////

  cv::Mat color_correction(cv::Mat src, int parameter){ // same for all the tasks

    std::vector<cv::Mat> lab_planes(3);
    cv::Mat dst, lab_image;

    cv::cvtColor(src, lab_image, CV_BGR2Lab);

    // Extract the L channel
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(parameter);

    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

    return image_clahe;

  }

////////////////////////////////////////////////////////////////////////////////////////////////////

  void denoise(cv::Mat &src, int i){ // may be needed in a task

    cv::Mat dstx;

    for (int j = 0; j < i; j++){
      bilateralFilter(src, dstx, 6, 8, 8);
      bilateralFilter(dstx, src, 6, 8, 8);
    }

  }

};

namespace post_processing{

  std_msgs::Float64MultiArray empty_contour_handler(cv::Point2f center_ideal){

    std_msgs::Float64MultiArray array;

    int x_cord = center_ideal.x - 320;
    int y_cord = 240 - center_ideal.y;

    if (x_cord < -270)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-2); // left_side
    }
    else if (x_cord > 270)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-3); // right_side
    }
    else if (y_cord > 200)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-3);  // top
    }
    else if (y_cord < -200)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-4);  // bottom
    }

    return array;

  }

  std_msgs::Float64MultiArray edge_case_handler(cv::Point2f center, int radius){

    std_msgs::Float64MultiArray array;

    int net_x_cord = 320 - center.x + radius;
    int net_y_cord = -240 + center.y + radius;

    if (net_x_cord < -310)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-2);  // right_side
    }
    else if (net_x_cord > 310)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-1);  // left_side
    }
    else if (net_y_cord > 230)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-3);  // bottom
    }
    else if (net_y_cord < -230)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-4);  // right_side
    }
    else if (radius > 110)
    {
      for (int i = 0; i < 4; i++)
        array.data.push_back(-5); // when the bot is very near
    }
    else
    {
      float distance;
      distance = pow(radius / 7526.5, -.92678);  // function found using experiment
      array.data.push_back(radius);                   // publish radius
      array.data.push_back((320 - center.x));
      array.data.push_back(-(240 - center.y));
      array.data.push_back(distance);
    }

    return array;

    }

    int get_largest_contour_index(std::vector<std::vector<cv::Point2f> > contours){ // to get the leargest contour index in a group of contours

      int largest_contour_index = 0;
      double largest_area = 0;

      for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
      {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
          largest_area = a;
          largest_contour_index = i;  // Store the index of largest contour
        }
      }

      return largest_contour_index;

    }

    cv::Point2f get_center_of_contour(std::vector<std::vector<cv::Point2f> > contours){ // to get the center of the contour

      int largest_contour_index = get_largest_contour_index(contours);
      std::vector<std::vector<cv::Point> > hull(1);
      cv::convexHull(cv::Mat(contours[largest_contour_index]), hull[0], false);

      cv::Moments mu;
      mu = cv::moments(hull[0], false);
      cv::Point2f center_of_mass;

      center_of_mass = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

      return center_of_mass;

    }

    void avg_fn(float *radius_ideal, std::vector<cv::Point2f> &center_ideal, float &current_radius, cv::Point2f &current_center, int &count_avg){
      float radius_avg = (radius_ideal[0] + radius_ideal[1] + radius_ideal[2] + radius_ideal[3] + radius_ideal[4]) / 5;
      if ((current_radius < (radius_avg + 10)) && (count_avg >= 5))
      {
        radius_ideal[4] = radius_ideal[3];
        radius_ideal[3] = radius_ideal[2];
        radius_ideal[2] = radius_ideal[1];
        radius_ideal[1] = radius_ideal[0];
        radius_ideal[0] = current_radius;
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = current_center;
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        radius_ideal[count_avg] = current_radius;
        center_ideal[count_avg] = current_center;
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }
    }

};

namespace task_buoy{

  void get_buoys_params(ros::NodeHandle &nh, int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2])
  {
    nh.getParam("buoy_detection/r1min", red_buoy[0][0]);
    nh.getParam("buoy_detection/r1max", red_buoy[0][1]);
    nh.getParam("buoy_detection/r2min", red_buoy[1][0]);
    nh.getParam("buoy_detection/r2max", red_buoy[1][1]);
    nh.getParam("buoy_detection/r3min", red_buoy[2][0]);
    nh.getParam("buoy_detection/r3max", red_buoy[2][1]);

    nh.getParam("buoy_detection/g1max", green_buoy[0][0]);
    nh.getParam("buoy_detection/g1min", green_buoy[0][1]);
    nh.getParam("buoy_detection/g2max", green_buoy[1][0]);
    nh.getParam("buoy_detection/g2min", green_buoy[1][1]);
    nh.getParam("buoy_detection/g3max", green_buoy[2][0]);
    nh.getParam("buoy_detection/g3min", green_buoy[2][1]);

    nh.getParam("buoy_detection/b1max", blue_buoy[0][0]);
    nh.getParam("buoy_detection/b1min", blue_buoy[0][1]);
    nh.getParam("buoy_detection/b2max", blue_buoy[1][0]);
    nh.getParam("buoy_detection/b2min", blue_buoy[1][1]);
    nh.getParam("buoy_detection/b3max", blue_buoy[2][0]);
    nh.getParam("buoy_detection/b3min", blue_buoy[2][1]);

  }

  void threshold(int BGR[][2], int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2], int flag)
  {
    if (flag == 0)
    {
      for (int i = 0; i < 3; i++){
        for (int j = 0; j < 2; j++){
          blue_buoy[i][j] = BGR[i][j];
        }
      }
    }

    else if (flag == 1)
    {
      for (int i = 0; i < 3; i++){
        for (int j = 0; j < 2; j++){
          green_buoy[i][j] = BGR[i][j];
        }
      }
    }

    else if (flag == 2)
    {
      for (int i = 0; i < 3; i++){
        for (int j = 0; j < 2; j++){
          red_buoy[i][j] = BGR[i][j];
        }
      }
    }
  }

  void set_buoy_params(ros::NodeHandle n, int red_buoy[][2], int blue_buoy[][2], int green_buoy[][2])
  {
    n.setParam("buoy_detection/r1min", red_buoy[0][0]);
    n.setParam("buoy_detection/r1max", red_buoy[0][1]);
    n.setParam("buoy_detection/r2min", red_buoy[1][0]);
    n.setParam("buoy_detection/r2max", red_buoy[1][1]);
    n.setParam("buoy_detection/r3min", red_buoy[2][0]);
    n.setParam("buoy_detection/r3max", red_buoy[2][1]);

    n.setParam("buoy_detection/g1max", green_buoy[0][0]);
    n.setParam("buoy_detection/g1min", green_buoy[0][1]);
    n.setParam("buoy_detection/g2max", green_buoy[1][0]);
    n.setParam("buoy_detection/g2min", green_buoy[1][1]);
    n.setParam("buoy_detection/g3max", green_buoy[2][0]);
    n.setParam("buoy_detection/g3min", green_buoy[2][1]);

    n.setParam("buoy_detection/b1max", blue_buoy[0][0]);
    n.setParam("buoy_detection/b1min", blue_buoy[0][1]);
    n.setParam("buoy_detection/b2max", blue_buoy[1][0]);
    n.setParam("buoy_detection/b2min", blue_buoy[1][1]);
    n.setParam("buoy_detection/b3max", blue_buoy[2][0]);
    n.setParam("buoy_detection/b3min", blue_buoy[2][1]);

  }

  void update_values(task_buoy::buoyConfig &config, int flag, int blue_buoy[][2], int green_buoy[][2], int red_buoy[][2])
  {

    if (flag == 0){
      config.t1min_param = blue_buoy[0][0];
      config.t1max_param = blue_buoy[0][1];
      config.t2min_param = blue_buoy[1][0];
      config.t2max_param = blue_buoy[1][1];
      config.t3min_param = blue_buoy[2][0];
      config.t3max_param = blue_buoy[2][1];
    }
    else if (flag == 1){
      config.t1min_param = green_buoy[0][0];
      config.t1max_param = green_buoy[0][1];
      config.t2min_param = green_buoy[1][0];
      config.t2max_param = green_buoy[1][1];
      config.t3min_param = green_buoy[2][0];
      config.t3max_param = green_buoy[2][1];
    }
    else if (flag == 2){
      config.t1min_param = red_buoy[0][0];
      config.t1max_param = red_buoy[0][1];
      config.t2min_param = red_buoy[1][0];
      config.t2max_param = red_buoy[1][1];
      config.t3min_param = red_buoy[2][0];
      config.t3max_param = red_buoy[2][1];
    }

  }

  void threshold_values_update(int BGR[][2], task_buoy::buoyConfig &config)
  {
    BGR[0][0] = config.t1min_param;
    BGR[0][1] = config.t1max_param;
    BGR[1][0] = config.t2min_param;
    BGR[1][1] = config.t2max_param;
    BGR[2][0] = config.t3min_param;
    BGR[2][1] = config.t3max_param;
  }

};
