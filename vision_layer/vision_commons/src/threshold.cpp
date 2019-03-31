#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include "threshold.h"

cv::Mat vision_commons::Threshold::threshold(cv::Mat &raw, int low_a, 
                                      int high_a, int low_b, int high_b, int low_c, int high_c)
{
  cv::Mat image_thresholded;
  if ((high_a > low_a && high_b > low_b && high_c > low_c))
    inRange(raw, cv::Scalar(low_a, low_b, low_c), cv::Scalar(high_a, high_b, high_c), image_thresholded);
  return image_thresholded;
}
