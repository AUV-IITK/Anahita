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
  static void filterContours (const std::vector<std::vector<cv::Point> >& contours, std::vector<int>& idx, double threshold);
  static std::vector<std::vector<cv::Point> > filterContours (const std::vector<std::vector<cv::Point> >& contours, double threshold);
  static void sortFromBigToSmall (std::vector<std::vector<cv::Point> >& contours);
};
} // namespace vision_commons

#endif
