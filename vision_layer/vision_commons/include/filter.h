#ifndef blue_filter_H
#define blue_filter_H

#include "opencv2/core/core.hpp"

namespace vision_commons
{
class Filter
{
public:
  static cv::Mat clahe(cv::Mat& image, double clahe_clip,
                    int clahe_grid_size);
  static cv::Mat balance_white(cv::Mat& image);
  static cv::Mat blue_filter(cv::Mat& image, double clahe_clip,
                            int clahe_grid_size, int clahe_bilateral_iter,
                            int balanced_bilateral_iter, double denoise_h);
};
} // namespace vision_commons

#endif
