#include "opencv2/photo/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "filter.h"

cv::Mat vision_commons::Filter::clahe(cv::Mat &image, double clahe_clip, int clahe_grid_size)
{
  cv::Mat lab_image;
  cv::cvtColor(image, lab_image, CV_BGR2Lab);
  std::vector<cv::Mat> lab_planes(3);
  cv::split(lab_image, lab_planes);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clahe_clip, cv::Size(clahe_grid_size, clahe_grid_size));
  cv::Mat l0dst;
  clahe->apply(lab_planes[0], l0dst);
  l0dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);
  cv::Mat clahed;
  cv::cvtColor(lab_image, clahed, CV_Lab2BGR);
  return clahed;
}

cv::Mat vision_commons::Filter::balance_white(cv::Mat &mat)
{
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3 * 256 * sizeof(int));

  for (int y = 0; y < mat.rows; ++y)
  {
    uchar *ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x)
    {
      for (int j = 0; j < 3; ++j)
      {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols * mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 255; ++j)
    {
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

  for (int y = 0; y < mat.rows; ++y)
  {
    uchar *ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x)
    {
      for (int j = 0; j < 3; ++j)
      {
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

cv::Mat vision_commons::Filter::blue_filter(
    cv::Mat &image,
    double clahe_clip,
    int clahe_grid_size,
    int clahe_bilateral_iter,
    int balanced_bilateral_iter,
    double denoise_h)
{

  if (!image.empty())
  {
		int64 t0_ = cv::getTickCount();
    cv::Mat blue_filtered = vision_commons::Filter::clahe(image, clahe_clip, clahe_grid_size);
    int64 t1_ = cv::getTickCount();
		// ROS_INFO("Time taken by CLAHE: %lf", (t1_-t0_)/cv::getTickFrequency());    
    cv::Mat temp = blue_filtered.clone();
    cv::Mat temp2;
    int64 t3_ = cv::getTickCount();
    for (int i = 0; i < clahe_bilateral_iter; i++)
    {
      cv::bilateralFilter(temp, temp2, 5, 8.0, 8.0);
      temp2.copyTo(temp);
    }
    int64 t4_ = cv::getTickCount();
		// ROS_INFO("Time taken by first CLAHE Bilateral: %lf", (t4_-t3_)/cv::getTickFrequency());    
    blue_filtered = vision_commons::Filter::balance_white(temp);
    int64 t5_ = cv::getTickCount();
    temp = blue_filtered.clone();
    for (int i = 0; i < balanced_bilateral_iter / 2; i++)
    {
      cv::bilateralFilter(temp, temp2, 6, 8.0, 8.0);
      temp2.copyTo(temp);
    }
    int64 t6_ = cv::getTickCount();
		// ROS_INFO("Time taken by second CLAHE Bilateral: %lf", (t6_-t5_)/cv::getTickFrequency());   

    /*
    int64 t7_ = cv::getTickCount();
    cv::fastNlMeansDenoisingColored(temp, blue_filtered, denoise_h, denoise_h, 7, 11);
    int64 t8_ = cv::getTickCount();
    ROS_INFO("Time taken by Denoising: %lf", (t8_-t7_)/cv::getTickFrequency());    
    */
    
    return blue_filtered;
  }
  else
    return image;
}
