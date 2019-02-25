#include "opencv2/photo/photo.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "geometry.h"

double vision_commons::Geometry::distance(cv::Point &p1, cv::Point &p2)
{
  double distance = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  return distance;
}

double vision_commons::Geometry::angleWrtY(cv::Point &p1, cv::Point &p2)
{
  double angle = atan2(p1.x - p2.x, p1.y - p2.y);
  return abs(angle * 180 / CV_PI);
}
