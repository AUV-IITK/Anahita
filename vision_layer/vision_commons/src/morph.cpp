#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include "morph.h"

cv::Mat vision_commons::Morph::open(cv::Mat &raw, int element_size, int element_centerX, int element_centerY, int iterations)
{
  cv::Mat opening_closing_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(element_size, element_size), cv::Point(element_centerX, element_centerY));
  cv::morphologyEx(raw, raw, cv::MORPH_OPEN, opening_closing_kernel, cv::Point(-1, -1), iterations);
  return raw;
}

cv::Mat vision_commons::Morph::close(cv::Mat &raw, int element_size, int element_centerX, int element_centerY, int iterations)
{
  cv::Mat opening_closing_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(element_size, element_size), cv::Point(element_centerX, element_centerY));
  cv::morphologyEx(raw, raw, cv::MORPH_CLOSE, opening_closing_kernel, cv::Point(-1, -1), iterations);
  return raw;
}
