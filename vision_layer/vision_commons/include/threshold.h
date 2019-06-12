#ifndef threshold_H
#define threshold_H
#include "opencv2/core/core.hpp"

namespace vision_commons
{
class Threshold
{
public:
  static cv::Mat threshold (cv::Mat& raw, int low_a, int high_a, int low_b, int high_b, int low_c, int high_c);
  static cv::Mat threshold (const cv::Mat& raw, const cv::Scalar& bgr_min, const cv::Scalar& bgr_max);
};
} // namespace vision_commons

#endif
