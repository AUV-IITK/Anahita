#ifndef contour_H
#define contour_H

#include "opencv2/core/core.hpp"

namespace vision_commons
{
class Contour
{
public:
  static std::vector<std::vector<cv::Point> > getBestX(cv::Mat &raw, int x);
  static std::vector<std::vector<cv::Point> > getBestXConvexHulled(cv::Mat &raw, int x);
  static std::vector<cv::Point> getLargestContour(cv::Mat &raw);
                                                                    
};
} // namespace vision_commons

#endif
