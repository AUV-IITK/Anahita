#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include <iostream>
#include "contour.h"

std::vector<std::vector<cv::Point> > vision_commons::Contour::getBestX(cv::Mat& raw, int x)
{
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(raw, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
  if (contours.size() != 0)
  {
    std::vector<float> areas;
    for (int i = 0; i < contours.size(); i++)
    {
      float area = cv::contourArea(contours[i]);
      if (area == 0.0)
        contours.erase(contours.begin() + i--);
      else
        areas.push_back(cv::contourArea(contours[i]));
    }
    float max_area = 0.0f;
    int max_index = 0;
    float area = 0.0f;
    int iter = 0;
    if (x < areas.size())
      iter = x;
    else
      iter = areas.size();
    for (int i = 0; i < iter; i++)
    {
      max_area = areas[i];
      max_index = i;
      for (int j = i + 1; j < contours.size(); j++)
      {
        area = areas[j];
        if (area > max_area)
        {
          max_area = area;
          max_index = j;
        }
      }
      std::vector<cv::Point> temp = contours[i];
      contours[i] = contours[max_index];
      contours[max_index] = temp;
      float tempArea = areas[i];
      areas[i] = areas[max_index];
      areas[max_index] = tempArea;
      max_area = 0.0;
      area = 0.0;
      max_index = 0;
    }

    if (areas.size() > x)
    {
      std::vector<std::vector<cv::Point> > topX(contours.begin(), contours.begin() + x);
      return topX;
    }
    else
      return contours;
  }
  else
    return contours;
}

std::vector<std::vector<cv::Point> > vision_commons::Contour::getBestXConvexHulled(cv::Mat& raw, int x)
{
  std::vector<std::vector<cv::Point> > contours = getBestX(raw, x);
  std::vector<std::vector<cv::Point> > hull(contours.size());
  for (int i = 0; i < contours.size(); i++)
  {
    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
  }
  return hull;
}
