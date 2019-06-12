#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <string>
#include "threshold.h"

cv::Mat vision_commons::Threshold::threshold(cv::Mat &raw, int low_a, 
                                            int high_a, int low_b, int high_b, 
                                            int low_c, int high_c)
{
  cv::Mat image_thresholded;
  if ((high_a > low_a && high_b > low_b && high_c > low_c))
    inRange(raw, cv::Scalar(low_a, low_b, low_c), cv::Scalar(high_a, high_b, high_c), image_thresholded);
  else 
    image_thresholded = raw.clone();
  return image_thresholded;
}

cv::Mat vision_commons::Threshold::threshold (const cv::Mat& raw, const cv::Scalar& bgr_min, const cv::Scalar& bgr_max) {
    cv::Mat img_thres;
    if (bgr_max[0] > bgr_min[0] && bgr_max[1] > bgr_min[1] && bgr_max[2] > bgr_min[2])
        inRange(raw, bgr_min, bgr_max, img_thres);
    else 
        img_thres = raw.clone();
    return img_thres;
}