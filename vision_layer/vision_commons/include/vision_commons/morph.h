#ifndef morph_H
#define morph_H

#include "opencv2/core/core.hpp"

namespace vision_commons
{
class Morph
{
public:
  static cv::Mat open(
      cv::Mat& raw,
      int element_size,
      int element_centerX,
      int element_centerY,
      int iterations);

  static cv::Mat close(
      cv::Mat& raw,
      int element_size,
      int element_centerX,
      int element_centerY,
      int iterations);
};
} // namespace vision_commons

#endif
